package frc.robot.subsystems;

import java.io.*;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import fi.iki.elonen.NanoHTTPD;

public class WebServer extends SubsystemBase {

    private static final Map<String, Double> dashboardValues = new ConcurrentHashMap<>();
    private static final Map<String, String> autoChooser = new ConcurrentHashMap<>();
    private static volatile String selectedAuto = "Taxi";

    static {
        autoChooser.put("Taxi", "Taxi");
    }

    public static void putNumber(String key, double value) {
        dashboardValues.put(key, value);
    }

    public static double getNumber(String key, double defaultVal) {
        return dashboardValues.getOrDefault(key, defaultVal);
    }

    public static String getSelectedAuto() {
        return selectedAuto;
    }

    public static Map<String, String> getAutos() {
        return autoChooser;
    }

    private final DashboardServer server;

    public WebServer() {
        server = new DashboardServer(2839);
        try {
            server.start(5000, false);
            System.out.println("Dashboard: http://" + getLocalIp() + ":2839/");
        } catch (IOException e) {
            stopServer();
        }
    }

    @Override
    public void periodic() {}

    public void stopServer() {
        if (server != null) server.stop();
    }

    private String getLocalIp() {
        try {
            return InetAddress.getLocalHost().getHostAddress();
        } catch (Exception e) {
            return "127.0.0.1";
        }
    }

    private static class DashboardServer extends NanoHTTPD {

        public DashboardServer(int port) {
            super(port);
        }

        @Override
        public Response serve(IHTTPSession session) {
            String uri = session.getUri();

            if ("/update".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                putNumber("musicPlaying", getParam(p, "playing"));
                putNumber("musicLevel", getParam(p, "level"));
                putNumber("musicBeat", getParam(p, "beat"));
                return newFixedLengthResponse("OK");
            }

            if ("/data".equals(uri)) {
                StringBuilder json = new StringBuilder("{");
                boolean first = true;
                for (var e : dashboardValues.entrySet()) {
                    if (!first) json.append(",");
                    json.append("\"").append(e.getKey()).append("\":").append(e.getValue());
                    first = false;
                }
                json.append("}");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }

            if ("/autos".equals(uri)) {
                StringBuilder json = new StringBuilder("{");
                json.append("\"selected\":\"").append(selectedAuto).append("\",");
                json.append("\"options\":{");
                boolean first = true;
                for (var e : autoChooser.entrySet()) {
                    if (!first) json.append(",");
                    json.append("\"").append(e.getKey()).append("\":\"").append(e.getValue()).append("\"");
                    first = false;
                }
                json.append("}}");
                Response r = newFixedLengthResponse(json.toString());
                r.addHeader("Content-Type", "application/json");
                return r;
            }

            if ("/setAuto".equals(uri)) {
                Map<String, List<String>> p = session.getParameters();
                String auto = p.getOrDefault("auto", List.of("DoNothing")).get(0);
                if (autoChooser.containsValue(auto)) selectedAuto = auto;
                return newFixedLengthResponse("OK");
            }

            try (InputStream is = getClass().getClassLoader().getResourceAsStream("dashboard.html")) {
                if (is == null) return newFixedLengthResponse("dashboard.html wasn't found.");
                return newFixedLengthResponse(Response.Status.OK, "text/html",
                        new String(is.readAllBytes(), StandardCharsets.UTF_8));
            } catch (IOException e) {
                return newFixedLengthResponse("Error");
            }
        }

        private double getParam(Map<String, List<String>> p, String key) {
            try {
                return Double.parseDouble(p.get(key).get(0));
            } catch (Exception e) {
                return 0;
            }
        }
    }
}