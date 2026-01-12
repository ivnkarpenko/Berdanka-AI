import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketTimeoutException;

DatagramSocket sock;
int UDP_PORT = 5005;

int cx = 240, cy = 160;
boolean ok = false;
float conf = 0.0;

float lerpX, lerpY;

final int W = 480;
final int H = 320;

final int boxSize = 20;
final int centerRangePx = 18;
final float smooth = 0.25;

long lastRxMs = 0;

void setup() {
  size(480, 320);
  surface.setTitle("Processing UDP Debug 480x320");

  try {
    sock = new DatagramSocket(UDP_PORT);
    sock.setSoTimeout(1); // не блокируем draw()
    println("Listening UDP on port " + UDP_PORT);
  } catch (Exception e) {
    println("UDP socket error: " + e);
  }

  lerpX = width/2;
  lerpY = height/2;
}

void draw() {
  background(15);

  // читаем все пакеты, что пришли (если несколько)
  readAllUdp();

  // сглаживание
  lerpX = lerp(lerpX, cx, smooth);
  lerpY = lerp(lerpY, cy, smooth);

  // рамка
  stroke(60);
  noFill();
  rectMode(CORNER);
  rect(0, 0, width-1, height-1);

  // крест центра
  stroke(255, 50);
  line(0, height/2, width, height/2);
  line(width/2, 0, width/2, height);

  boolean inCenter =
    abs(lerpX - width/2) <= centerRangePx &&
    abs(lerpY - height/2) <= centerRangePx;

  // цвет квадрата
  strokeWeight(2);
  if (ok) {
    if (inCenter) stroke(0, 255, 0);
    else stroke(255, 0, 0);
  } else {
    stroke(160);
  }

  rectMode(CENTER);
  noFill();
  rect(lerpX, lerpY, boxSize, boxSize);

  line(lerpX - 4, lerpY, lerpX + 4, lerpY);
  line(lerpX, lerpY - 4, lerpX, lerpY + 4);
  strokeWeight(1);

  // инфа
  fill(255);
  textSize(12);
  textAlign(LEFT, TOP);

  text("cx=" + cx + " cy=" + cy, 8, 8);
  text("ok=" + (ok ? "1" : "0") + " conf=" + nf(conf, 0, 2), 8, 24);

  long age = millis() - lastRxMs;
  String link = (lastRxMs == 0) ? "WAIT" : ((age > 600) ? "NO DATA" : "LIVE");
  text("udp=" + UDP_PORT + " " + link, 8, 40);
}

void readAllUdp() {
  if (sock == null) return;

  byte[] buf = new byte[256];
  DatagramPacket pkt = new DatagramPacket(buf, buf.length);

  while (true) {
    try {
      sock.receive(pkt);
      String line = new String(pkt.getData(), 0, pkt.getLength());
      line = trim(line);
      parseXY(line);
      lastRxMs = millis();
    } catch (SocketTimeoutException te) {
      break; // пакетов больше нет
    } catch (Exception e) {
      break;
    }
  }
}

void parseXY(String line) {
  // ожидаем: XY:241,156;OK:1;C:0.83
  if (!line.startsWith("XY:")) return;

  try {
    int semi1 = line.indexOf(";OK:");
    int semi2 = line.indexOf(";C:");

    String xyPart = (semi1 > 0) ? line.substring(3, semi1) : line.substring(3);
    String okPart = (semi1 > 0 && semi2 > semi1) ? line.substring(semi1+4, semi2) : "0";
    String cPart  = (semi2 > 0) ? line.substring(semi2+3) : "0.0";

    String[] xy = split(xyPart, ",");
    if (xy.length >= 2) {
      cx = constrain(int(xy[0]), 0, W-1);
      cy = constrain(int(xy[1]), 0, H-1);
    }

    ok = (int(okPart) == 1);
    conf = float(cPart);
  } catch (Exception e) {
    // ignore
  }
}

void stop() {
  if (sock != null) sock.close();
  super.stop();
}
