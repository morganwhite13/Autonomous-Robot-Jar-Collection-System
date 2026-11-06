import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.LongSupplier;

/** Console logging with a simulated-time prefix, e.g. {@code [  84.3s] Turning to 271}. */
final class Log {
  private Log() {}

  private static LongSupplier clockMs = () -> 0L;
  private static Consumer<String> sink = System.out::println;

  static void useClock(LongSupplier simulatedMillis) {
    clockMs = simulatedMillis;
  }

  /** Redirect output (tests silence or capture the log this way). */
  static void useSink(Consumer<String> lineSink) {
    sink = lineSink;
  }

  static void info(String message) {
    sink.accept(String.format(Locale.ROOT, "[%7.1fs] %s", clockMs.getAsLong() / 1000.0, message));
  }

  static void warn(String message) {
    info("WARNING: " + message);
  }

  static void phase(String title) {
    info("=== " + title + " ===");
  }

  /** Multi-line block printed without a time prefix (used for the mission report). */
  static void block(String text) {
    for (String line : text.split("\n", -1)) {
      sink.accept(line);
    }
  }
}
