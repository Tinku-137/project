#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <math.h>
#include "matfun.h"
#include "geofun.h"

AsyncWebServer server(80);

const char* ssid = "Tinku";
const char* password = "password";

const char* input_parameter00 = "input00";
const char* input_parameter01 = "input01";
const char* input_parameter10 = "input10";
const char* input_parameter11 = "input11";
const char* input_parameter20 = "input20";
const char* input_parameter21 = "input21";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Triangle Properities</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Times New Roman; display: inline-block; text-align: center;}
    h2 {font-size: 2.0rem; color: blue;}
  </style> 
  </head><body>
  <h2>Medians</h2> 
  <p>Enter the values of points A, B, C
  <form action="/get">
    Vertex A (x1 y1): <input type="number" name="input00"><br><input type="number" name="input01"><br>
    Vertex B (x2 y2): <input type="number" name="input10"><br><input type="number" name="input11"><br>
    Vertex C (x3 y3): <input type="number" name="input20"><br><input type="number" name="input21"><br>
    <input type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connecting...");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response;

    double **A, **B, **C; // declare vertices
    double **D, **E, **F; // mid_points
    double **m1, **m2, **m3, **m4, **m5, **m6; // direction vectors
    double **n1, **n2, **n3, **n4, **n5, **n6; // normal vectors
    double c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12; // constant vectors
    double **t_vec, **c_vec, **mt_vec, **mc_vec, **at_vec, **alc_vec, **plc_vec;
    double a, b, c, pv, mv, nv; // triangle sides
    double **cx1, **cy1;
    double **D3, **E3, **F3;
    double **a_n1, **a_n2, **a_n3, **at_nor;
    double x1, x2, x3, **at_c_m;
    int ax, ay, bx, by, cx, cy, m = 2, n = 1, i, j;
    // Initialize matrix to zeros
    double **mat = createMat(2, 3);
    double **dir_vec = createMat(2, 3);
    double **norm_vec = createMat(2, 3);
    double **l_vec = createMat(1, 3);
    double **s_vec = createMat(1, 3);
    double **m_vec = createMat(3, 3);
    double **midpt_vec = createMat(2, 3);
    double **mdir_vec = createMat(2, 3);
    double **mnorm_vec = createMat(2, 3);
    double **ml_vec = createMat(1, 3);
    double **mm_vec = createMat(3, 3);
    double **centroid = createMat(1, 2);
    double **anorm_vec = createMat(2, 3);
    double **ac_vec = createMat(1, 3);
    double **am_vec = createMat(3, 3);
    double **o_center = createMat(1, 2);
    double **pc_vec = createMat(1, 3);
    double **pm_vec = createMat(3, 3);
    double **circum_c = createMat(1, 2);
    double **ss_vec = createMat(1, 3);
    double **dir_Mat = createMat(3, 3);
    double **a_dir = createMat(2, 3);
    double **a_nor = createMat(2, 3);
    double **a_c_m = createMat(1, 3);
    double **aam_vec = createMat(3, 3);
    double **in_cen = createMat(1, 2);
    double **adir_Mat = createMat(3, 3);
    double **aa_dir = createMat(2, 3);
    // Parse input values
    ax = request->arg(input_parameter00).toFloat();
    ay = request->arg(input_parameter01).toFloat();
    bx = request->arg(input_parameter10).toFloat();
    by = request->arg(input_parameter11).toFloat();
    cx = request->arg(input_parameter20).toFloat();
    cy = request->arg(input_parameter21).toFloat();

    mat[0][0] = ax;
    mat[1][0] = ay;
    mat[0][1] = bx; // Fixed typo
    mat[1][1] = by; // Fixed typo
    mat[0][2] = cx; // Fixed typo
    mat[1][2] = cy; // Fixed typo

    // Print vertices matrix
    A = Matcol(mat, m, 0);
    B = Matcol(mat, m, 1);
    C = Matcol(mat, m, 2);
    response += "Vertices Matrix = <br>";
    response += printMatToString(mat, 2, 3);

    // Direction vectors
    m1 = Matsub(A, B, m, 1);
    m2 = Matsub(B, C, m, 1);
    m3 = Matsub(C, A, m, 1);
    dir_vec = matrix_merge(m1, m2, m3, 2, 3);
    response += "Direction Matrix = <br>";
    response += printMatToString(dir_vec, 2, 3);

    // Normal vectors
    n1 = normVec(m1);
    n2 = normVec(m2);
    n3 = normVec(m3);
    norm_vec = matrix_merge(n1, n2, n3, 2, 3);
    response += "Normal Matrix = <br>";
    response += printMatToString(norm_vec, 2, 3);

    // Line constants
    c1 = Matdot(n1, A, 2);
    c2 = Matdot(n2, B, 2);
    c3 = Matdot(n3, C, 2);
    l_vec = matrix(c1, c2, c3);
    response += "Constant Matrix = <br>";
    response += printMatToString(l_vec, 1, 3);

    // Sides lengths
    a = Matnorm(m2, m);
    b = Matnorm(m3, m);
    c = Matnorm(m1, m);
    s_vec = matrix(a, b, c);
    response += "Distance:<br> " + String(a, 2) + " , " + String(b, 2) + " , " + String(c, 2) + " <br>";

    // Line Matrix
    t_vec = transposeMat(norm_vec, 2, 3);
    c_vec = transposeMat(l_vec, 1, 3);
    m_vec = matrix_2Merge(t_vec, 3, 2, c_vec, 3, 1);
    response += "Line matrix = <br>";
    response += printMatToString(m_vec, 3, 3);

    // Midpoint matrix
    D = Matsec(C, B, 2, 1);
    E = Matsec(A, C, 2, 1);
    F = Matsec(A, B, 2, 1);
    midpt_vec = matrix_merge(D, E, F, 2, 3);
    response += "Midpoint Matrix = <br>";
    response += printMatToString(midpt_vec, 2, 3);

    // Direction vectors
    m4 = Matsub(A, D, m, 1);
    m5 = Matsub(B, E, m, 1);
    m6 = Matsub(C, F, m, 1);
    mdir_vec = matrix_merge(m4, m5, m6, 2, 3);
    response += "Median Direction Matrix = <br>";
    response += printMatToString(mdir_vec, 2, 3);

    // Normal vectors
    n4 = normVec(m4);
    n5 = normVec(m5);
    n6 = normVec(m6);
    mnorm_vec = matrix_merge(n4, n5, n6, 2, 3);
    response += "Median Normal Matrix = <br>";
    response += printMatToString(mnorm_vec, 2, 3);

    // Line constants
    c4 = Matdot(n4, D, 2);
    c5 = Matdot(n5, E, 2);
    c6 = Matdot(n6, F, 2);
    ml_vec = matrix(c4, c5, c6);
    response += "Median Constant Matrix = <br>";
    response += printMatToString(ml_vec, 1, 3);

    // centroid
    cx1 = Matcol(transposeMat(mnorm_vec, 2, 3), m, 0);
    cy1 = Matcol(transposeMat(mnorm_vec, 2, 3), m, 1);

    // Line Matrix
    mt_vec = transposeMat(mnorm_vec, 2, 3);
    mc_vec = transposeMat(ml_vec, 1, 3);
    mm_vec = matrix_2Merge(mt_vec, 3, 2, mc_vec, 3, 1);

    centroid = line_intersect(mm_vec, 3, 3);
    response += "Centroid = <br>";
    response += printMatToString(centroid, 1, 2);

    // Normal Matrix
    anorm_vec = matrix_merge(m2, m3, m1, 2, 3);
    response += "Altitude Normal Matrix = <br>";
    response += printMatToString(anorm_vec, 2, 3);

    // constant Matrix
    c7 = Matdot(m2, A, 2);
    c8 = Matdot(m3, B, 2);
    c9 = Matdot(m1, C, 2);
    ac_vec = matrix(c7, c8, c9);
    response += "Altitude Constant Matrix = <br>";
    response += printMatToString(ac_vec, 1, 3);

    at_vec = transposeMat(anorm_vec, 2, 3);
    alc_vec = transposeMat(ac_vec, 1, 3);
    am_vec = matrix_2Merge(at_vec, 3, 2, alc_vec, 3, 1);
    response += "Altitude Line matrix = <br>";
    response += printMatToString(am_vec, 3, 3);

    o_center = line_intersect(am_vec, 3, 3);
    response += "Orthocenter = <br>";
    response += printMatToString(o_center, 1, 2);

    c10 = Matdot(m2, D, 2);
    c11 = Matdot(m3, E, 2);
    c12 = Matdot(m1, F, 2);

    pc_vec = matrix(c10, c11, c12);
    response += "Perpendicular Constant Matrix = <br>";
    response += printMatToString(pc_vec, 1, 3);

    plc_vec = transposeMat(pc_vec, 1, 3);
    pm_vec = matrix_2Merge(at_vec, 3, 2, plc_vec, 3, 1);
    response += "Perpendicular Line matrix = <br>";
    response += printMatToString(pm_vec, 3, 3);

    circum_c = line_intersect(pm_vec, 3, 3);
    response += "Circumcentre = <br>";
    response += printMatToString(circum_c, 1, 2);

    pv = ((c + b) - a) / 2;
    mv = ((a + c) - b) / 2;
    nv = ((a + b) - c) / 2;
    ss_vec = matrix(pv, mv, nv);

    response += "p,m,n Values = <br>";
    response += printMatToString(ss_vec, 1, 3);

    dir_Mat[0][0] = -1;
    dir_Mat[0][1] = nv / b;
    dir_Mat[0][2] = mv / c;
    dir_Mat[1][0] = nv / a;
    dir_Mat[1][1] = -1;
    dir_Mat[1][2] = pv / c;
    dir_Mat[2][0] = mv / a;
    dir_Mat[2][1] = pv / b;
    dir_Mat[2][2] = -1;

    a_dir = Matmul(mat, dir_Mat, 2, 3, 3);
    response += "Angular Bisector Direction Matrix = <br>";
    response += printMatToString(a_dir, 2, 3);

    // Angular Normal matrix
    a_n1 = normVec(Matcol(a_dir, 2, 0));
    a_n2 = normVec(Matcol(a_dir, 2, 1));
    a_n3 = normVec(Matcol(a_dir, 2, 2));
    a_nor = matrix_merge(a_n1, a_n2, a_n3, 2, 3);
    response += "Angular Bisector Normal matrix = <br>";
    response += printMatToString(a_nor, 2, 3);

    // constant matrix
    x1 = Matdot(a_n1, A, 2);
    x2 = Matdot(a_n2, B, 2);
    x3 = Matdot(a_n3, C, 2);
    a_c_m = matrix(x1, x2, x3);
    response += "Angular Bisector constant matrix= <br>";
    response += printMatToString(a_c_m, 1, 3);

    // printing line matrix
    at_nor = transposeMat(a_nor, 2, 3);
    at_c_m = transposeMat(a_c_m, 1, 3);
    aam_vec = matrix_2Merge(at_nor, 3, 2, at_c_m, 3, 1);
    response += "Angular Bisector Line matrix = <br>";
    response += printMatToString(aam_vec, 3, 3);

    // Angular intersection points
    in_cen = line_intersect(aam_vec, 3, 3);
    response += "Angular Bisector Intersection points = <br>";
    response += printMatToString(in_cen, 1, 2);

    // Angular Contact points
    adir_Mat[0][0] = 0;
    adir_Mat[0][1] = nv / b;
    adir_Mat[0][2] = mv / c;
    adir_Mat[1][0] = nv / a;
    adir_Mat[1][1] = 0;
    adir_Mat[1][2] = pv / c;
    adir_Mat[2][0] = mv / a;
    adir_Mat[2][1] = pv / b;
    adir_Mat[2][2] = 0;
    aa_dir = Matmul(mat, adir_Mat, 2, 3, 3);
    response += "Angular Bisector Contact Points = <br>";
    response += printMatToString(aa_dir, 2, 3);

    // End of the calculation
    response += "**************************** THE END ********************************** <br>";

    response += "<a href=\"/\">Return to Home Page</a>";

    request->send(200, "text/html", response);
  });

  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  // Nothing to do here for now
}

