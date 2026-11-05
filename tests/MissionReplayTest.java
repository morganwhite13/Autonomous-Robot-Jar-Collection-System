import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

class MissionReplayTest {

  private static String svg;
  private static Document doc;

  @BeforeAll
  static void render() throws Exception {
    Log.useSink(line -> { });
    svg = new MissionReplay().render(WorldFile.read(Path.of("worlds/ProjectWorld2025.wbt")));
    doc = DocumentBuilderFactory.newInstance().newDocumentBuilder()
        .parse(new ByteArrayInputStream(svg.getBytes(StandardCharsets.UTF_8)));
  }

  @Test
  void everyAnimationSharesOneDurationSoTheyStayInSync() {
    Set<String> durations = new HashSet<>();
    for (String tag : new String[] {"animate", "animateTransform"}) {
      NodeList nodes = doc.getElementsByTagName(tag);
      for (int i = 0; i < nodes.getLength(); i++) {
        Element e = (Element) nodes.item(i);
        durations.add(e.getAttribute("dur"));
        assertEquals("indefinite", e.getAttribute("repeatCount"));
      }
    }
    assertEquals(1, durations.size(), durations.toString());
  }

  @Test
  void animatedValueListsHaveOneEntryPerSample() {
    Set<Integer> counts = new HashSet<>();
    NodeList transforms = doc.getElementsByTagName("animateTransform");
    for (int i = 0; i < transforms.getLength(); i++) {
      counts.add(((Element) transforms.item(i)).getAttribute("values").split(";").length);
    }
    NodeList animates = doc.getElementsByTagName("animate");
    for (int i = 0; i < animates.getLength(); i++) {
      Element e = (Element) animates.item(i);
      if (e.getAttribute("attributeName").equals("stroke-dashoffset")) {
        counts.add(e.getAttribute("values").split(";").length);
      }
    }
    assertEquals(1, counts.size(), counts.toString());
    assertEquals(4, transforms.getLength()); // move and turn, for each robot
  }

  @Test
  void showsTheMeasuredOutcome() {
    assertTrue(svg.contains(">0/5 jars, stuck from 47 s<"), "v1 status");
    assertTrue(svg.contains(">v1 stuck from 47 s</text>"), "v1 label");
    assertTrue(svg.contains("pushing against a wooden box"), "v1 cause");
    assertTrue(svg.contains(">5/5 jars, done at 894 s<"), "v2 status");
  }

  @Test
  void withoutAnimationTheFinalStateIsShown() {
    // Static attributes (what a viewer that doesn't animate shows) are the end of the run.
    NodeList texts = doc.getElementsByTagName("text");
    int visibleStatuses = 0;
    for (int i = 0; i < texts.getLength(); i++) {
      Element t = (Element) texts.item(i);
      if (t.getTextContent().contains("/5 jars") && t.getAttribute("opacity").equals("1")) {
        visibleStatuses++;
      }
    }
    assertEquals(2, visibleStatuses);
    NodeList lines = doc.getElementsByTagName("polyline");
    for (int i = 0; i < lines.getLength(); i++) {
      assertEquals("0", ((Element) lines.item(i)).getAttribute("stroke-dashoffset"));
    }
  }
}
