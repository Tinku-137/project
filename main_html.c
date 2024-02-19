#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include "matfun.h"


#define PORT 1323 // Change at every runtime
#define BUFFER_SIZE 4096

void sendHTMLForm(int client_fd, double **mat, double **dir_vec, double **norm_vec, double **l_vec, double **s_vec, double **m_vec,  double **midpt_vec, double **mdir_vec,double **mnorm_vec, double **ml_vec, double **mm_vec, double **centroid, double **anorm_vec,double **ac_vec,double **am_vec, double **o_center, double **pc_vec, double **pm_vec, double **circum_c, double **ss_vec, double **a_dir, double **a_nor, double **a_c_m, double **aam_vec, double **in_cen, double **aa_dir) {
                  
     char html_response[8192]; 
      snprintf(html_response, sizeof(html_response),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: text/html\r\n\r\n"
             "<html>"
             "<head>"
             "<title>Math operations</title>"
             "</head>"
             "<body style=text-align:center>"
             "<h3>Give Inputs here </h3>"
             "<form action=\"/\" method=\"POST\" >"
             "<label for=\"Ax\">Point A - X:</label>"
             "<input type=\"text\" name=\"Ax\" value=\"%.2lf\"><br><br><br>"
             "<label for=\"Ay\">Point A - Y:</label>"
             "<input type=\"text\" name=\"Ay\" value=\"%.2lf\"><br><br><br>"
             "<label for=\"Bx\">Point B - X:</label>"
             "<input type=\"text\" name=\"Bx\" value=\"%.2lf\"><br><br><br>"
             "<label for=\"By\">Point B - Y:</label>"
             "<input type=\"text\" name=\"By\" value=\"%.2lf\"><br><br><br>"
             "<label for=\"Cx\">Point C - X:</label>"
             "<input type=\"text\" name=\"Cx\" value=\"%.2lf\"><br><br><br>"
             "<label for=\"Cy\">Point C - Y:</label>"
             "<input type=\"text\" name=\"Cy\" value=\"%.2lf\"><br><br><br>"
             "<input type=\"submit\" value=\"Calculate\">"
             "</form>"
                          "<h2>vertices Matrix = </h2>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h2>VECTORS</h2>"
                          "<h4>Direction Matrix</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>normal Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>constant Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>distanve Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>line Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h2>Median</h2>"
                          "<h4>midpoints Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>direction Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>normal Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>constant Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>line Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>centroid=</h4>"
                          "<p>%.4lf %.4lf</p>"
                           "<h2>Altitude</h2>"
                          "<h4>normal Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>constant Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>line Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>Orthocenter=</h4>"
                          "<p>%.4lf %.4lf</p>"
                           "<h2>perpendicular Bisector</h2>"
                          "<h4>constant Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>line Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>circumcenter=</h4>"
                          "<p>%.4lf %.4lf</p>"
                          "<h2>angular Bisector</h2>"
                          "<h4>p,m,n values =</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>direction Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>normal Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>constant Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<h4>line Matrix=</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                           "<h4>Intersection points=</h4>"
                          "<p>%.4lf %.4lf</p>"
                          
                          "<h4>contact points =</h4>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                          "<p>%.4lf %.4lf %.4lf</p>"
                           "</body>\n"
                          "</html>\n",
             // Pass matrix values here
             mat[0][0], mat[1][0], mat[0][1],
             mat[1][1], mat[0][2], mat[1][2],
             //
             mat[0][0], mat[1][0], mat[0][1],
             mat[1][1], mat[0][2], mat[1][2],
             // Vertices Matrix
             dir_vec[0][0], dir_vec[1][0], dir_vec[0][1],
             dir_vec[1][1], dir_vec[0][2], dir_vec[1][2],
             // Vertices Matrix
             norm_vec[0][0], norm_vec[1][0], norm_vec[0][1],
             norm_vec[1][1], norm_vec[0][2], norm_vec[1][2],
             //constant Matrix
              l_vec[0][0], l_vec[0][1], l_vec[0][2],
             //Distance Matrix
              s_vec[0][0], s_vec[0][1], s_vec[0][2],
              //line Matrix
              m_vec[0][0], m_vec[0][1], m_vec[0][2],
              m_vec[1][0], m_vec[1][1], m_vec[1][2],
              m_vec[2][0], m_vec[2][1], m_vec[2][2],
              //median
               //midpoinrMatrix
              midpt_vec[0][0], midpt_vec[1][0], midpt_vec[0][1],
             midpt_vec[1][1], midpt_vec[0][2], midpt_vec[1][2],
              //direction Matrix
              mdir_vec[0][0], mdir_vec[1][0], mdir_vec[0][1],
             mdir_vec[1][1], mdir_vec[0][2], mdir_vec[1][2],
             //normal Matrix
              mnorm_vec[0][0], mnorm_vec[1][0], mnorm_vec[0][1],
             mnorm_vec[1][1], mnorm_vec[0][2], mnorm_vec[1][2],
             
             //constant Matrix
              ml_vec[0][0], ml_vec[0][1], ml_vec[0][2],
             
             //line Matrix
              mm_vec[0][0], mm_vec[0][1], mm_vec[0][2],
              mm_vec[1][0], mm_vec[1][1], mm_vec[1][2],
              mm_vec[2][0], mm_vec[2][1], mm_vec[2][2],
              
              //centroid
              centroid[0][0], centroid[0][1],
              
               //normal Matrix
              anorm_vec[0][0], anorm_vec[1][0], anorm_vec[0][1],
             anorm_vec[1][1], anorm_vec[0][2], anorm_vec[1][2],
             
             //constant Matrix
              ac_vec[0][0], ac_vec[0][1], ac_vec[0][2],
             //line Matrix
              am_vec[0][0], am_vec[0][1], am_vec[0][2],
              am_vec[1][0], am_vec[1][1], am_vec[1][2],
              am_vec[2][0], am_vec[2][1], am_vec[2][2],
              
              //orthocenter
              o_center[0][0], o_center[0][1],
              //perpen
              //constant Matrix
              pc_vec[0][0], pc_vec[0][1], pc_vec[0][2],
              
              //line Matrix
              pm_vec[0][0], pm_vec[0][1], pm_vec[0][2],
              pm_vec[1][0], pm_vec[1][1], pm_vec[1][2],
              pm_vec[2][0], pm_vec[2][1], pm_vec[2][2],
              
              //circumcentercenter
              circum_c[0][0], circum_c[0][1],
              
              //p,m,n
              ss_vec[0][0], ss_vec[0][1], ss_vec[0][2],
              
               //direction Matrix
              a_dir[0][0], a_dir[1][0], a_dir[0][1],
             a_dir[1][1], a_dir[0][2], a_dir[1][2],
             //normal Matrix
              a_nor[0][0], a_nor[1][0], a_nor[0][1],
             a_nor[1][1], a_nor[0][2], a_nor[1][2],
             
             //constant Matrix
              a_c_m[0][0], a_c_m[0][1], a_c_m[0][2],
              
               //line Matrix
              aam_vec[0][0], aam_vec[0][1], aam_vec[0][2],
              aam_vec[1][0], aam_vec[1][1], aam_vec[1][2],
              aam_vec[2][0], aam_vec[2][1], aam_vec[2][2],
              
               //intersection points
              in_cen[0][0], in_cen[0][1],
              
              //contact points
              aa_dir[0][0], aa_dir[1][0], aa_dir[0][1],
             aa_dir[1][1], aa_dir[0][2], aa_dir[1][2]
              
             );
 
    write(client_fd, html_response, strlen(html_response));
}            
             
 int  main()
{
    int server_fd, client_fd;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    char buffer[BUFFER_SIZE] = {0};
    
double **A,**B,**C;//declare vertices
double **D,**E,**F;//mid_points
double **m1,**m2,**m3,**m4,**m5,**m6;//direction vectors
double **n1,**n2,**n3,**n4,**n5,**n6;//normal vectors
double **temp;//temporary array
double c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12;//constant vectors
double **t_vec,**c_vec,**mt_vec,**mc_vec,
       **at_vec,**alc_vec,**plc_vec;
double a,b,c,pv,mv,nv;//triangle sides
double **cx1,**cy1;
double **D3,**E3,**F3;
double **a_n1,**a_n2,**a_n3,**at_nor;
double x1,x2,x3,**at_c_m;
int m =2, n=3, i, j;
// Initialize matrix to zeros
double **mat = createMat(2, 3);
double **dir_vec = createMat(2,3);
double **norm_vec= createMat(2,3);
double **l_vec =createMat(1,3);
double **s_vec= createMat(1,3);
double **m_vec=createMat(3,3);
double **midpt_vec=createMat(2,3);
double **mdir_vec=createMat(2,3);
double **mnorm_vec=createMat(2,3);
double **ml_vec=createMat(1,3);
double **mm_vec=createMat(3,3);
double **centroid=createMat(1,2);
double **anorm_vec=createMat(2,3);
double **ac_vec=createMat(1,3);
double **am_vec=createMat(3,3);
double **o_center=createMat(1,2);
double **pc_vec=createMat(1,3);
double **pm_vec=createMat(3,3);
double **circum_c=createMat(1,2);
double **ss_vec=createMat(1,3);
double **dir_Mat= createMat(3,3);
double **a_dir=createMat(2,3);
double **a_nor=createMat(2,3);
double **a_c_m=createMat(1,3);
double **aam_vec=createMat(3,3);
double **in_cen=createMat(1,2);
double **adir_Mat= createMat(3,3);
double **aa_dir=createMat(2,3);

  // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(1234);

    // Forcefully attaching socket to the port
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

 // Print server address
    printf("Server is listening on %s:%d\n", inet_ntoa(address.sin_addr), ntohs(address.sin_port));


while (1) {
    if ((client_fd = accept(server_fd, (struct sockaddr*)&address, &addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    printf("New connection accepted\n");

    read(client_fd, buffer, BUFFER_SIZE);
    printf("Received data from client: %s\n", buffer);
    char *ptr_Ax = strstr(buffer, "Ax=");
    char *ptr_Ay = strstr(buffer, "Ay=");
    char *ptr_Bx = strstr(buffer, "Bx=");
    char *ptr_By = strstr(buffer, "By=");
    char *ptr_Cx = strstr(buffer, "Cx=");
    char *ptr_Cy = strstr(buffer, "Cy=");

    
    if (!mat) {
        perror("Error creating matrix");
        exit(EXIT_FAILURE);
    }

    if (ptr_Ax && ptr_Ay && ptr_Bx && ptr_By && ptr_Cx && ptr_Cy) {
        // Extracting vertices if all points are found
        sscanf(ptr_Ax, "Ax=%lf", &mat[0][0]);
        sscanf(ptr_Ay, "Ay=%lf", &mat[1][0]);
        sscanf(ptr_Bx, "Bx=%lf", &mat[0][1]);
        sscanf(ptr_By, "By=%lf", &mat[1][1]);
        sscanf(ptr_Cx, "Cx=%lf", &mat[0][2]);
        sscanf(ptr_Cy, "Cy=%lf", &mat[1][2]);

        // Print vertices matrix
         A = Matcol(mat,m,0);
         B = Matcol(mat,m,1);
         C = Matcol(mat,m,2);
        printf("Vertices Matrix = \n");
        printMat(mat, 2, 3);
        printf("\n");
        //Direction vectors
        m1 = Matsub(A,B,m,1);
        m2 = Matsub(B,C,m,1);
        m3 = Matsub(C,A,m,1);
        dir_vec=matrix_merge(m1,m2,m3,2,3);
        printf("Direction Matrix = \n");
        printMat(dir_vec,2,3);
        printf("\n");
        //Normal vectors
        n1 = normVec(m1);
        n2 = normVec(m2);
        n3 = normVec(m3);
        norm_vec=matrix_merge(n1,n2,n3,2,3);
        printf("Normal Matrix = \n");
        printMat(norm_vec,2,3);
        printf("\n");
        //Line constants
        c1 = Matdot(n1,A,2);
        c2 = Matdot(n2,B,2);
        c3 = Matdot(n3,C,2);
        l_vec = matrix(c1,c2,c3);
        printf("Constant Matrix = \n");
        printMat(l_vec,1,3);
        printf("\n");
        //Sides lengths
        a = Matnorm(m2,m);
        b = Matnorm(m3,m);
        c = Matnorm(m1,m);
        s_vec = matrix(a,b,c);
        printf("Distance matrix = \n");
        printMat(s_vec,1,3);
        printf("\nDistance:\n %lf , %lf ,%lf \n",a,b,c);
        //Line Matrix
        t_vec =transposeMat(norm_vec,2,3);
        //printMat(t_vec,3,2);
        c_vec =transposeMat(l_vec,1,3);
        //printMat(c_vec,3,1);
        m_vec =matrix_2Merge(t_vec,3, 2,c_vec, 3, 1);
        //printf("Line matrix = \n");
        printMat(m_vec, 3, 3);
        //printf("\n");
        //Midpoint matrix
        D= Matsec(C,B,2,1);
        //printMat(D,2,1);

        E= Matsec(A,C,2,1);
        //printMat(E,2,1);

        F= Matsec(A,B,2,1);
        //printMat(F,2,1);

        midpt_vec=matrix_merge(D,E,F,2,3);
        printf("Midpoint Matrix = \n");
        printMat(midpt_vec,2,3);
        printf("\n");
        
        //Direction vectors
        m4 = Matsub(A,D,m,1);
        m5 = Matsub(B,E,m,1);
        m6 = Matsub(C,F,m,1);
        mdir_vec=matrix_merge(m4,m5,m6,2,3);
        printf("Median Direction Matrix = \n");
        printMat(mdir_vec,2,3);
        printf("\n");
        
        //Normal vectors
        n4 = normVec(m4);
        n5 = normVec(m5);
        n6 = normVec(m6);
       mnorm_vec=matrix_merge(n4,n5,n6,2,3);
       printf("Median Normal Matrix = \n");
       printMat(mnorm_vec,2,3);
       printf("\n");

       //Line constants
       c4 = Matdot(n4,D,2);
       c5 = Matdot(n5,E,2);
       c6 = Matdot(n6,F,2);
       ml_vec = matrix(c4,c5,c6);
       printf("Median Constant Matrix = \n");
       printf("%lf %lf %lf \n",c4,c5,c6);
       printMat(ml_vec,1,3);
       printf("\n");
       
       //centroid
       cx1=Matcol(transposeMat(mnorm_vec,2,3),m,0);
       cy1=Matcol(transposeMat(mnorm_vec,2,3),m,1);

       //Line Matrix
       mt_vec =transposeMat(mnorm_vec,2,3);
       //printMat(t_vec,3,2);
       mc_vec =transposeMat(ml_vec,1,3);
       //printMat(c_vec,3,1);
       mm_vec =matrix_2Merge(mt_vec,3, 2,mc_vec, 3, 1);
       //printf("Median Line matrix = \n");
       printMat(mm_vec, 3, 3);
       //printf("\n");
       
       centroid=line_intersect(mm_vec,3,3);
       printf("Centroid = \n");
       printMat(centroid,1,2);
       printf("\n");
         printf("**************************** Altitude********************************** \n");

        //Normal Matrix
        //printf("Altitude Normal Matrix = \n");
        anorm_vec= matrix_merge(m2,m3,m1,2,3);
        printMat(anorm_vec,2,3);
       //printf("\n");
       
       //constant Matrix
        c7 = Matdot(m2,A,2);
        c8 = Matdot(m3,B,2);
        c9 = Matdot(m1,C,2);
        ac_vec = matrix(c7,c8,c9);
        printf("Altitude Constant Matrix = \n");
        printf("%lf %lf %lf \n",c7,c8,c9);
        printMat(ac_vec,1,3);
        printf("\n");
        
       at_vec =transposeMat(anorm_vec,2,3);
       //printMat(at_vec,3,2);
       alc_vec =transposeMat(ac_vec,1,3);
       //printMat(alc_vec,3,1);
       //merging two into one
       am_vec =matrix_2Merge(at_vec,3, 2,alc_vec, 3, 1);
       //printf("Altitude Line matrix = \n");
       printMat(am_vec, 3, 3);
       //printf("\n");
       
        o_center=line_intersect(am_vec,3,3);
        printf("Orthocenter = \n");
        printMat(o_center,1,2);
        printf("\n");
        
        printf("**************************** Perpendicular Bisector ********************************** \n");
        c10 = Matdot(m2,D,2);
        c11 = Matdot(m3,E,2);
        c12 = Matdot(m1,F,2);

        pc_vec = matrix(c10,c11,c12);
        printf("Perpendicular Constant Matrix = \n");
        //printf("%lf %lf %lf \n",c10,c11,c12);
        printMat(pc_vec,1,3);
        printf("\n");

        //merging two matrices into oneprintf("\nIncentre = \n");
        plc_vec = transposeMat(pc_vec,1,3);
        pm_vec = matrix_2Merge(at_vec,3,2,plc_vec,3,1);
        //printf("Perpendicular Line matrix = \n");
        printMat(pm_vec, 3, 3);
        //printf("\n");
       circum_c=line_intersect(pm_vec,3,3);
       printf("Circumcentre = \n");
       printMat(circum_c,1,2);
       printf("\n");
       printf("**************************** Angular Bisector ********************************** \n");
        pv = ((c+b)-a)/2;
        mv = ((a+c)-b)/2;
        nv = ((a+b)-c)/2;
        ss_vec = matrix(pv,mv,nv); 
        printf("p,m,n Values = \n");
        printMat(ss_vec,1,3);
        printf("\n"); 
        printf("Angular Bisector Direction Matrix = \n");
        
        dir_Mat[0][0]=  -1 ;      
        dir_Mat[0][1]= nv/b;    
        dir_Mat[0][2]= mv/c;
        dir_Mat[1][0]= nv/a;
        dir_Mat[1][1]= -1 ;
        dir_Mat[1][2]= pv/c;
        dir_Mat[2][0]= mv/a;
        dir_Mat[2][1]= pv/b;
        dir_Mat[2][2]= -1 ;

        a_dir=Matmul(mat,dir_Mat,2,3,3);
        printMat(a_dir,2,3);
         printf("\n");
         
         //Angular Normal matrix
         printf("Angular Bisector Normal matrix = \n");

         a_n1=normVec(Matcol(a_dir,2,0));
         a_n2=normVec(Matcol(a_dir,2,1));
         a_n3=normVec(Matcol(a_dir,2,2));
         a_nor=matrix_merge(a_n1,a_n2,a_n3,2,3);
         printMat(a_nor,2,3);
         printf("\n");
         
         //constant matrix
         printf("Angular Bisector constant matrix= \n");
         x1 = Matdot(a_n1,A,2);
         x2 = Matdot(a_n2,B,2);
         x3 = Matdot(a_n3,C,2);
         a_c_m=matrix(x1,x2,x3);
         printMat(a_c_m,1,3);
         printf("\n");
         
         
         //printing line matrix
         at_nor =transposeMat(a_nor,2,3);
         at_c_m = transposeMat(a_c_m,1,3);
         aam_vec = matrix_2Merge(at_nor,3,2,at_c_m,3,1);
         printf("Angular Bisector Line matrix = \n");
         printMat(aam_vec, 3, 3);
         printf("\n");
         
         
         //Angular intersection points
         in_cen=line_intersect(aam_vec,3,3);
         printf("Angular Bisector Intersection points = \n");
         printMat(in_cen,1,2);
         printf("\n");

        //Angular Contact points
         
         adir_Mat[0][0]= 0;
         adir_Mat[0][1]= nv/b;    
         adir_Mat[0][2]= mv/c;
         adir_Mat[1][0]= nv/a;
         adir_Mat[1][1]= 0;
         adir_Mat[1][2]= pv/c;
         adir_Mat[2][0]= mv/a;
         adir_Mat[2][1]= pv/b;
         adir_Mat[2][2]= 0;
         aa_dir=Matmul(mat,adir_Mat,2,3,3);
         printf("Angular Bisector Contact Points = \n");
         printMat(aa_dir,2,3);
         printf("\n");
         printf("**************************** THE END ********************************** \n");

        
//printf("\n");
        sendHTMLForm(client_fd, mat, dir_vec, norm_vec, l_vec, s_vec, m_vec, midpt_vec, mdir_vec,mnorm_vec,ml_vec, mm_vec,centroid, anorm_vec,ac_vec,am_vec,o_center,pc_vec,pm_vec, circum_c, ss_vec, a_dir,a_nor, a_c_m, aam_vec,in_cen,aa_dir );
    } else {
        printf("One or more points not found in the buffer.\n");
        // Send HTML form without proper data
        sendHTMLForm(client_fd, mat, dir_vec, norm_vec, l_vec, s_vec, m_vec, midpt_vec, mdir_vec, mnorm_vec,ml_vec, mm_vec,centroid, anorm_vec,ac_vec,am_vec,o_center,pc_vec,pm_vec, circum_c, ss_vec, a_dir, a_nor, a_c_m, aam_vec, in_cen, aa_dir);
    }

    // Free matrix memory
    
    
    
    close(client_fd);
}

return 0;

}            
             
             
             
             
             
             
