import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.ErroneousTree;
import com.sun.source.tree.ClassTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.util.DocTreeScanner;
import com.sun.source.util.DocTrees;
import com.sun.source.util.JavacTask;
import com.sun.source.util.TreePathScanner;
import com.sun.source.util.Trees;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.ToolProvider;

/** Parse named main-source types and attached JavaDoc without resolving Android dependencies. */
class AuthorshipDocs {
    static String quote(String text) {
        return "\"" + text.replace("\\", "\\\\").replace("\"", "\\\"")
                .replace("\r", "\\r").replace("\n", "\\n").replace("\t", "\\t") + "\"";
    }

    public static void main(String[] args) throws Exception {
        Path root = Path.of(args.length == 0 ? "." : args[0]).toAbsolutePath().normalize();
        List<Path> paths = new ArrayList<>();
        for (String module : List.of("TeamCode", "FtcRobotController")) {
            try (var stream = Files.walk(root.resolve(module + "/src/main/java"))) {
                stream.filter(p -> p.toString().endsWith(".java")).sorted().forEach(paths::add);
            }
        }
        JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
        DiagnosticCollector<JavaFileObject> diagnostics = new DiagnosticCollector<>();
        var manager = compiler.getStandardFileManager(diagnostics, null, java.nio.charset.StandardCharsets.UTF_8);
        JavacTask task = (JavacTask) compiler.getTask(null, manager, diagnostics,
                List.of("-proc:none", "-encoding", "UTF-8"), null,
                manager.getJavaFileObjectsFromPaths(paths));
        Trees trees = Trees.instance(task);
        DocTrees docs = DocTrees.instance(task);
        List<String> rows = new ArrayList<>();
        for (CompilationUnitTree unit : task.parse()) {
            String source = unit.getSourceFile().getCharContent(true).toString();
            String path = root.relativize(Path.of(unit.getSourceFile().toUri())).toString().replace('\\', '/');
            new TreePathScanner<Void, Void>() {
                final ArrayDeque<String> owners = new ArrayDeque<>();
                @Override public Void visitClass(ClassTree node, Void unused) {
                    String simple = node.getSimpleName().toString();
                    if (simple.isEmpty()) return super.visitClass(node, unused);
                    String name = String.join(".", owners) + (owners.isEmpty() ? "" : ".") + simple;
                    long start = trees.getSourcePositions().getStartPosition(unit, node);
                    DocCommentTree doc = docs.getDocCommentTree(getCurrentPath());
                    long docStart = -1, docEnd = -1;
                    List<String> errors = new ArrayList<>();
                    if (doc != null) {
                        long bodyStart = docs.getSourcePositions().getStartPosition(unit, doc, doc);
                        if (bodyStart >= 0) {
                            docStart = source.lastIndexOf("/**", (int) bodyStart);
                            docEnd = source.indexOf("*/", (int) bodyStart) + 2;
                        }
                        new DocTreeScanner<Void, Void>() {
                            @Override public Void visitErroneous(ErroneousTree error, Void p) {
                                errors.add(quote(error.getDiagnostic().getMessage(null)));
                                return null;
                            }
                        }.scan(doc, null);
                    }
                    rows.add("{\"path\":" + quote(path) + ",\"name\":" + quote(name)
                            + ",\"top_level\":" + owners.isEmpty() + ",\"start\":" + start
                            + ",\"doc_start\":" + docStart + ",\"doc_end\":" + docEnd
                            + ",\"doc_errors\":[" + String.join(",", errors) + "]}");
                    owners.addLast(simple);
                    super.visitClass(node, unused);
                    owners.removeLast();
                    return null;
                }
            }.scan(unit, null);
        }
        System.out.println("[" + String.join(",\n", rows) + "]");
        for (var diagnostic : diagnostics.getDiagnostics()) {
            if (diagnostic.getKind() == Diagnostic.Kind.ERROR) {
                System.err.println(diagnostic);
                throw new IllegalStateException("Main-source parsing failed");
            }
        }
        manager.close();
    }
}
