package com.makeyourpet.chicaserver.protocol;

import android.util.Log;
import com.makeyourpet.chicaserver.control.ChicaController;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;

public final class OriginalTcpControlServer {
    public static final int PORT = 18711;
    private static final String TAG = "ChicaServer";
    private static final String ACK = "ack";
    private static final String BYE = "bye";
    private static final String TRACE = "trace";
    private static final String READY = "ready:";
    private static final String BUSY = "busy:";
    private static final String TRACE_PREFIX = "trace:";

    private final ChicaController controller;
    private volatile boolean running;
    private ServerSocket serverSocket;

    public OriginalTcpControlServer(ChicaController controller) {
        this.controller = controller;
    }

    public void start() {
        if (running) return;
        running = true;
        Thread thread = new Thread(this::serveForever, "chica-original-tcp");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        running = false;
        closeServerSocket();
    }

    private void serveForever() {
        while (running) {
            try {
                serveOnce();
                Thread.sleep(1000L);
            } catch (Exception error) {
                if (running) Log.e(TAG, "original tcp server failed", error);
            }
        }
    }

    private void serveOnce() throws Exception {
        try (ServerSocket listener = new ServerSocket(PORT)) {
            serverSocket = listener;
            try (Socket client = listener.accept()) {
                handleClient(client);
            }
        } finally {
            serverSocket = null;
        }
    }

    private void handleClient(Socket socket) throws Exception {
        socket.setSoTimeout(1000);
        try (PrintWriter writer = new PrintWriter(socket.getOutputStream(), true);
                BufferedReader reader = new BufferedReader(
                        new InputStreamReader(socket.getInputStream()))) {
            int ackCount = 0;
            writer.println(READY + controller.originalStatusString());
            while (running) {
                String command;
                try {
                    command = reader.readLine();
                } catch (SocketTimeoutException timeout) {
                    continue;
                }
                if (command == null || BYE.equals(command)) {
                    controller.requestOriginalClientStop();
                    return;
                }
                controller.logControl("tcp_recv", command);
                if (ChicaController.DEVELOPER_FIXTURES && TRACE.equals(command)) {
                    writer.println(TRACE_PREFIX + controller.lastMathTrace());
                    continue;
                }
                if (!ACK.equals(command)) ackCount = 0;
                if (controller.isBusy()) {
                    Thread.sleep(100L);
                    writer.println(BUSY + controller.originalStatusString());
                    continue;
                }
                if (ACK.equals(command) && controller.shouldHandleOriginalAck()) {
                    ackCount++;
                    controller.handleOriginalAck(ackCount);
                    Thread.sleep(100L);
                    writer.println(READY + controller.originalStatusString());
                } else {
                    controller.submitOriginalCommand(command);
                    writer.println(READY + controller.originalStatusString());
                }
            }
        }
    }

    private void closeServerSocket() {
        try {
            if (serverSocket != null) serverSocket.close();
        } catch (Exception ignored) {
        }
    }
}
