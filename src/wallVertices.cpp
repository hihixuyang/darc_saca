#include "wallVertices.h"

void buildObstacles(std::vector<Vertex>& vertices,
										std::vector<Eigen::Vector3f>& normals, int sim, int scene) {
  double t = 0.12;
  double r = 0.282;
  if (sim) {
    if (scene == 1) {
      // Floor
      vertices[0].tr <<  40.0, -40.0,  0.0; vertices[0].tl <<  40.0,  40.0,  0.0;
      vertices[0].br << -40.0, -40.0,  0.0; vertices[0].bl << -40.0,  40.0,  0.0; 
      normals[0] <<  0.0,   0.0,   1.0;
      // Wall 0
      vertices[1].tr << 3-t, -3, 3.6; vertices[1].tl << 3-t, 3, 3.6;
      vertices[1].br << 3-t, -3, 0.0; vertices[1].bl << 3-t, 3, 0.0;
      normals[1]  << -1.0,   0.0,   0.0;
      // Wall 1
      vertices[2].tr << 3, 3-t, 3.6; vertices[2].tl << -3, 3-t, 3.6;
      vertices[2].br << 3, 3-t, 0.0; vertices[2].bl << -3, 3-t, 0.0;
      normals[2]  <<  0.0,  -1.0,   0.0;
      // Wall 2
      vertices[3].tr << -3+t, 3, 3.6; vertices[3].tl << -3+t, -3, 3.6;
      vertices[3].br << -3+t, 3, 0.0; vertices[3].bl << -3+t, -3, 0.0;
      normals[3]  <<  1.0,   0.0,   0.0;
      // Wall 3
      vertices[4].tr << -3, -3+t, 3.6; vertices[4].tl << 3, -3+t, 3.6;
      vertices[4].br << -3, -3+t, 0.0; vertices[4].bl << 3, -3+t, 0.0;
      normals[4]  <<  0.0,   1.0,   0.0;
      // Wall 0 Back
      vertices[5].tr << 3+t, 3, 3.6; vertices[5].tl << 3+t, -3, 3.6;
      vertices[5].br << 3+t, 3, 0.0; vertices[5].bl << 3+t, -3, 0.0;
      normals[5]  <<  1.0,   0.0,   0.0;
      // Wall 1 Back
      vertices[6].tr << -3, 3+t, 3.6; vertices[6].tl << 3, 3+t, 3.6;
      vertices[6].br << -3, 3+t, 0.0; vertices[6].bl << 3, 3+t, 0.0;
      normals[6]  <<  0.0,   1.0,   0.0;
      // Wall 2 back
      vertices[7].tr << -3-t, -3, 3.6; vertices[7].tl << -3-t, 3, 3.6;
      vertices[7].br << -3-t, -3, 0.0; vertices[7].bl << -3-t, 3, 0.0;
      normals[7]  << -1.0,   0.0,   0.0;  
      // Wall 3 back
      vertices[8].tr << 3, -3-t, 3.6; vertices[8].tl << -3, -3-t, 3.6;
      vertices[8].br << 3, -3-t, 0.0; vertices[8].bl << -3, -3-t, 0.0;
      normals[8]  <<  0.0,  -1.0,   0.0;
    } else if (scene == 2) {
      // Floor
      vertices[0].tr <<  40.0, -40.0,  0.0; vertices[0].tl <<  40.0,  40.0,  0.0;
      vertices[0].br << -40.0, -40.0,  0.0; vertices[0].bl << -40.0,  40.0,  0.0; 
      normals[0]     <<  0.0,   0.0,   1.0;
      // Wall 0
      vertices[1].tr << -3, t, 3.6; vertices[1].tl <<  3, t, 3.6;
      vertices[1].br << -3, t, 0.0; vertices[1].bl <<  3, t, 0.0; 
      normals[1]     <<  0.0,   1.0,   0.0;
      // Wall 1 
      vertices[2].tr <<  3, -t, 3.6; vertices[2].tl << -3, -t, 3.6;
      vertices[2].br <<  3, -t, 0.0; vertices[2].bl << -3, -t, 0.0; 
      normals[2]     <<  0.0,  -1.0,   0.0;
      // Wall 2
      vertices[3].tr << t, 3, 3.6; vertices[3].tl << t, -3, 3.6;
      vertices[3].br << t, 3, 0.0; vertices[3].bl << t, -3, 0.0;
      normals[3]     <<  1.0,   0.0,   0.0;
      // Wall 3
      vertices[4].tr << -t, -3, 3.6; vertices[4].tl << -t, 3, 3.8;
      vertices[4].br << -t, -3, 0.0; vertices[4].bl << -t, 3, 0.0;
      normals[4]     << -1.0,   0.0,   0.0;
      // Wall 4
      vertices[5].tr << 3, t, 3.6; vertices[5].tl << 3, -t, 3.6;
      vertices[5].br << 3, t, 0.0; vertices[5].bl << 3, -t, 0.0;
      normals[5]     <<  1.0,   0.0,   0.0;
      // Wall 5
      vertices[6].tr << -t, 3, 3.6; vertices[6].tl << t, 3, 3.6;
      vertices[6].br << -t, 3, 0.0; vertices[6].bl << t, 3, 0.0;
      normals[6]     <<  0.0,   1.0,   0.0; 
      // Wall 6
      vertices[7].tr << -3, -t, 3.6; vertices[7].tl << -3, t, 3.6;
      vertices[7].br << -3, -t, 0.0; vertices[7].bl << -3, t, 0.0;
      normals[7]     << -1.0,   0.0,   0.0;
      // Wall 7
      vertices[8].tr << t, -3, 3.6; vertices[8].tl << -t, -3, 3.6; 
      vertices[8].br << t, -3, 0.0; vertices[8].bl << -t, -3, 0.0;
      normals[8]     <<  0.0,  -1.0,   0.0;
    }  else if (scene == 3) {
      t = 0.06;
      Eigen::Vector2f A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S;
      A <<  5.0, -6.0; 
      B << -4.0, -6.0; 
      C <<  2.0, -6.0; 
      D <<  2.0,  0.0;
      E <<  5.0, -3.5; 
      F <<  5.0,  3.5; 
      G <<  1.0,  3.5; 
      H << -4.0,  3.5;
      I <<  1.0,  1.5; 
      J <<  3.0,  2.0; 
      K <<  3.0,  0.0; 
      L << -1.0,  2.0;
      M << -1.0,  0.0; 
      N <<  0.0,  0.0; 
      O <<  0.0, -4.0; 
      P << -2.0,  0.0;
      Q << -2.0, -4.0;
      R << -2.0, -2.0;
      S << -4.0, -2.0;
      // Wall 0
      vertices[0].tr << B(0), B(1)+t, 2.4; vertices[0].tl << A(0), A(1)+t, 2.4;
      vertices[0].br << B(0), B(1)+t, 0.0; vertices[0].bl << A(0), A(1)+t, 0.0;
      normals[0] << 0.0, 1.0, 0.0;
      // Wall 0 Back
      vertices[1].tr << A(0), A(1)-t, 2.4; vertices[1].tl << B(0), B(1)-t, 2.4;
      vertices[1].br << A(0), A(1)-t, 0.0; vertices[1].bl << B(0), B(1)-t, 0.0;
      normals[1] << 0.0, -1.0, 0.0;
      // Wall 0 End
      vertices[2].tr << A(0), A(1)+t, 2.4; vertices[2].tl << A(0), A(1)-t, 2.4;
      vertices[2].br << A(0), A(1)+t, 0.0; vertices[2].bl << A(0), A(1)-t, 0.0;
      normals[2] << 1.0, 0.0, 0.0;
      // Wall 1 
      vertices[3].tr << E(0)-t, E(1), 2.4; vertices[3].tl << F(0)-t, F(1), 2.4;
      vertices[3].br << E(0)-t, E(1), 0.0; vertices[3].bl << F(0)-t, F(1), 0.0;
      normals[3] << -1.0, 0.0, 0.0;
      // Wall 1 back
      vertices[4].tr << F(0)+t, F(1), 2.4; vertices[4].tl << E(0)+t, E(1), 2.4;
      vertices[4].br << F(0)+t, F(1), 0.0; vertices[4].bl << E(0)+t, E(1), 0.0;
      normals[4] << 1.0, 0.0, 0.0;
      // Wall 1 End
      vertices[5].tr << E(0)+t, E(1), 2.4; vertices[5].tl << E(0)-t, E(1), 2.4;
      vertices[5].br << E(0)+t, E(1), 0.0; vertices[5].bl << E(0)-t, E(1), 0.0;
      normals[5] << 0.0, -1.0, 0.0;
      // Wall 2 
      vertices[6].tr << F(0), F(1)-t, 2.4; vertices[6].tl << H(0), H(1)-t, 2.4;
      vertices[6].br << F(0), F(1)-t, 0.0; vertices[6].bl << H(0), H(1)-t, 0.0;
      normals[6] << 0.0, -1.0, 0.0;
      // Wall 2 Back
      vertices[7].tr << H(0), H(1)+t, 2.4; vertices[7].tl << F(0), F(1)+t, 2.4;
      vertices[7].br << H(0), H(1)+t, 0.0; vertices[7].bl << F(0), F(1)+t, 0.0;
      normals[7] << 0.0, 1.0, 0.0;
      // Wall 3
      vertices[8].tr << H(0)+t, H(1), 2.4; vertices[8].tl << B(0)+t, B(1), 2.4;
      vertices[8].br << H(0)+t, H(1), 0.0; vertices[8].bl << B(0)+t, B(1), 0.0;
      normals[8] << 1.0, 0.0, 0.0;
      // Wall 3 Back
      vertices[9].tr << B(0)-t, B(1), 2.4; vertices[9].tl << H(0)-t, H(1), 2.4;
      vertices[9].br << B(0)-t, B(1), 0.0; vertices[9].bl << H(0)-t, H(1), 0.0;
      normals[9] << -1.0, 0.0, 0.0;
      // Wall 4
      vertices[10].tr << J(0)+t, J(1), 2.4; vertices[10].tl << K(0)+t, K(1), 2.4;
      vertices[10].br << J(0)+t, J(1), 0.0; vertices[10].bl << K(0)+t, K(1), 0.0;
      normals[10] << 1.0, 0.0, 0.0;
      // Wall 4 Back
      vertices[11].tr << K(0)-t, K(1), 2.4; vertices[11].tl << J(0)-t, J(1), 2.4;
      vertices[11].br << K(0)-t, K(1), 0.0; vertices[11].bl << J(0)-r, J(1), 0.0;
      normals[11] << -1.0, 0.0, 0.0;
      // Wall 4 End
      vertices[12].tr << J(0)-t, J(1), 2.4; vertices[12].tl << J(0)+t, J(1), 2.4;
      vertices[12].br << J(0)-t, J(1), 0.0; vertices[12].bl << J(0)+t, J(1), 0.0;
      normals[12] << 0.0, 1.0, 0.0;
      // Wall 5
      vertices[13].tr << G(0)+t, G(1), 2.4; vertices[13].tl << I(0)+t, I(1), 2.4;
      vertices[13].br << G(0)+t, G(1), 0.0; vertices[13].bl << I(0)+t, I(1), 0.0;
      normals[13] << 1.0, 0.0, 0.0;
      // Wall 5 Back
      vertices[14].tr << I(0)-t, I(1), 2.4; vertices[14].tl << G(0)-t, G(1), 2.4;
      vertices[14].br << I(0)-t, I(1), 0.0; vertices[14].bl << G(0)-t, G(1), 0.0;
      normals[14] << -1.0, 0.0, 0.0;
      // Wall 5 End
      vertices[15].tr << I(0)+t, I(1), 2.4; vertices[15].tl << I(0)-t, I(1), 2.4;
      vertices[15].br << I(0)+t, I(1), 0.0; vertices[15].bl << I(0)-t, I(1), 0.0;
      normals[15] << 0.0, -1.0, 0.0;
      // Wall 6
      vertices[16].tr << L(0)+t, L(1), 2.4; vertices[16].tl << M(0)+t, M(1), 2.4;
      vertices[16].br << L(0)+t, L(1), 0.0; vertices[16].bl << M(0)+t, M(1), 0.0;
      normals[16] << 1.0, 0.0, 0.0;
      // Wall 6 Back
      vertices[17].tr << M(0)-t, M(1), 2.4; vertices[17].tl << L(0)-t, L(1), 2.4;
      vertices[17].br << M(0)-t, M(1), 0.0; vertices[17].bl << L(0)-t, L(1), 0.0;
      normals[17] << -1.0, 0.0, 0.0;
      // Wall 6 End
      vertices[18].tr << L(0)-t, L(1), 2.4; vertices[18].tl << L(0)+t, L(1), 2.4;
      vertices[18].br << L(0)-t, L(1), 0.0; vertices[18].bl << L(0)+t, L(1), 0.0;
      normals[18] << 0.0, 1.0, 0.0;
      // Wall 7
      vertices[19].tr << K(0), K(1)-t, 2.4; vertices[19].tl << P(0), P(1)-t, 2.4;
      vertices[19].br << K(0), K(1)-t, 0.0; vertices[19].bl << P(0), P(1)-t, 0.0;
      normals[19] << 0.0, -1.0, 0.0;
      // Wall 7 Back
      vertices[20].tr << P(0), P(1)+t, 2.4; vertices[20].tl << K(0), K(1)+t, 2.4;
      vertices[20].br << P(0), P(1)+t, 0.0; vertices[20].bl << K(0), K(1)+t, 0.0;
      normals[20] << 0.0, 1.0, 0.0;
      // Wall 7 End
      vertices[21].tr << P(0), P(1)-t, 2.4; vertices[21].tl << P(0), P(1)+t, 2.4;
      vertices[21].br << P(0), P(1)-t, 0.0; vertices[21].bl << P(0), P(1)+t, 0.0;
      normals[21] << -1.0, 0.0, 0.0;
      // Wall 8
      vertices[22].tr << D(0)+t, D(1), 2.4; vertices[22].tl << C(0)+t, C(1), 2.4;
      vertices[22].br << D(0)+t, D(1), 0.0; vertices[22].bl << C(0)+t, C(1), 0.0;
      normals[22] << 1.0, 0.0, 0.0;
      // Wall 8 Back
      vertices[23].tr << C(0)-t, C(1), 2.4; vertices[23].tl << D(0)-t, D(1), 2.4;
      vertices[23].br << C(0)-t, C(1), 0.0; vertices[23].bl << D(0)-t, D(1), 0.0;
      normals[23] << -1.0, 0.0, 0.0;
      // Wall 9 
      vertices[24].tr << N(0)+t, N(1), 2.4; vertices[24].tl << O(0)+t, O(1), 2.4;
      vertices[24].br << N(0)+t, N(1), 0.0; vertices[24].bl << O(0)+t, O(1), 0.0;
      normals[24] << 1.0, 0.0, 0.0;
      // Wall 9 Back
      vertices[25].tr << O(0)-t, O(1), 2.4; vertices[25].tl << N(0)-t, N(1), 2.4;
      vertices[25].br << O(0)-t, O(1), 0.0; vertices[25].bl << N(0)-t, N(1), 0.0;
      normals[25] << -1.0, 0.0, 0.0;
      // Wall 9 End
      vertices[26].tr << O(0)+t, O(1), 2.4; vertices[26].tl << O(0)-t, O(1), 2.4;
      vertices[26].br << O(0)+t, O(1), 0.0; vertices[26].bl << O(0)-t, O(1), 0.0;
      normals[26] << 0.0, -1.0, 0.0;
      // Wall 10 
      vertices[27].tr << O(0), O(1)-t, 2.4; vertices[27].tl << Q(0), Q(1)-t, 2.4;
      vertices[27].br << O(0), O(1)-t, 0.0; vertices[27].bl << Q(0), Q(1)-t, 0.0;
      normals[27] << 0.0, -1.0, 0.0;
      // Wall 10 Back
      vertices[28].tr << Q(0); Q(1)+t, 2.4; vertices[28].tl << O(0), O(1)+t, 2.4;
      vertices[28].br << Q(0); Q(1)+t, 0.0; vertices[28].bl << O(0), O(1)+t, 0.0;
      normals[28] << 0.0, 1.0, 0.0;
      // Wall 10 End
      vertices[29].tr << Q(0), Q(1)-t, 2.4; vertices[29].tl << Q(0), Q(1)+t, 2.4;
      vertices[29].br << Q(0), Q(1)-t, 0.0; vertices[29].bl << Q(0), Q(1)-t, 0.0;
      normals[29] << 1.0, 0.0, 0.0;
      // Wall 11
      vertices[30].tr << R(0), R(1)-t, 2.4; vertices[30].tl << S(0), S(1)-t, 2.4;
      vertices[30].br << R(0), R(1)-t, 0.0; vertices[30].bl << S(0), S(1)-t, 0.0;
      normals[30] << 0.0, -1.0, 0.0;
      // Wall 11 Back
      vertices[31].tr << S(0), S(1)+t, 2.4; vertices[31].tl << R(0), R(1)+t, 2.4;
      vertices[31].br << S(0), S(1)+t, 0.0; vertices[31].bl << R(0), R(1)+t, 0.0;
      normals[31] << 0.0, 1.0, 0.0;
      // Wall 11 End
      vertices[32].tr << R(0), R(1)+t, 2.4; vertices[32].tl << R(0), R(1)-t, 2.4;
      vertices[32].br << R(0), R(1)+t, 0.0; vertices[32].tl << R(0), R(1)-t, 0.0;
      normals[32] << 1.0, 0.0, 0.0;
      // Floor
      vertices[33].tr << 40.0, -40.0, 0.0; vertices[33].tl << 40.0, 40.0, 0.0;
      vertices[33].br << -40.0, -40.0, 0.0; vertices[33].bl << -40.0, 40.0, 0.0;
      normals[33] << 0.0, 0.0, 1.0; 
    } else if (scene == 4) {
      t = 0.0625;
      double h = 1.2;
      double l = 2.0;

			Eigen::Vector2f In0, In1, In2, In3, In4, In5, In6, In7, In8;
      double theta = -M_PI/6.0; // 30 deg
      In0 << -1.125, 8.0;
      In1 << 1.125, 6.0;
      In2 << -1.125, 4.0;
      In3 << 1.125, 2.0;
      In4 << -1.125, 0.0;
      In5 << 1.125, -2.0;
      In6 << -1.125, -4.0;
      In7 << 1.125, -6.0;
      In8 << -1.125, -8.0;
      
      Eigen::Vector2f rb, lb, rt, lt;
      rb << -l, t;
      lb << l, t; 
      rt << -l, t; 
      lt << l, t;
      
      Eigen::Rotation2Df rot2b(theta);
      Eigen::Rotation2Df rot2t(-theta);
      rb = rot2b*rb; 
      lb = rot2b*lb;
      rt = rot2t*rt;
      lt = rot2t*lt;
      
      Eigen::Vector2f r0, l0, r1, l1, r2, l2, r3, l3, r4, l4, r5, l5, r6, l6, r7, l7, r8, l8;
      r0 = In0 + rb; 
      l0 = In0 + lb;
      r1 = In1 + rt; 
      l1 = In1 + lt; 
      r2 = In2 + rb; 
      l2 = In2 + lb;
      r3 = In3 + rt; 
      l3 = In3 + lt;
      r4 = In4 + rb;
      l4 = In4 + lb;
      r5 = In5 + rt;
      l5 = In5 + lt;
      r6 = In6 + rb;
      l6 = In6 + lb;
      r7 = In7 + rt;
      l7 = In7 + lt;
      r8 = In8 + rb; 
      l8 = In8 + lb;
      
      // Inside Wall 0
      vertices[0].tr << r0[0], r0[1], 2.4; vertices[0].tl << l0[0], l0[1], 2.4;
      vertices[0].br << r0[0], r0[1], 0.0; vertices[0].bl << l0[0], l0[1], 0.0;
      // Inside wall 1
      vertices[1].tr << r1[0], r1[1], 2.4; vertices[1].tl << l1[0], l1[1], 2.4;
      vertices[1].br << r1[0], r1[1], 0.0; vertices[1].bl << l1[0], l1[1], 0.0;
      // Inside wall 2
      vertices[2].tr << r2[0], r2[1], 2.4; vertices[2].tl << l2[0], l2[1], 2.4;
      vertices[2].br << r2[0], r2[1], 0.0; vertices[2].bl << l2[0], l2[1], 0.0;
      // Inside wall 3
      vertices[3].tr << r3[0], r3[1], 2.4; vertices[3].tl << l3[0], l3[1], 2.4;
      vertices[3].br << r3[0], r3[1], 0.0; vertices[3].bl << l3[0], l3[1], 0.0;
      // Inside wall 4
      vertices[4].tr << r4[0], r4[1], 2.4; vertices[4].tl << l4[0], l4[1], 2.4;
      vertices[4].br << r4[0], r4[1], 0.0; vertices[4].bl << l4[0], l4[1], 0.0;
      // Inside wall 5
      vertices[5].tr << r5[0], r5[1], 2.4; vertices[5].tl << l5[0], l5[1], 2.4;
      vertices[5].br << r5[0], r5[1], 0.0; vertices[5].bl << l5[0], l5[1], 0.0;
      // Inside wall 6
      vertices[6].tr << r6[0], r6[1], 2.4; vertices[6].tl << l6[0], l6[1], 2.4;
      vertices[6].br << r6[0], r6[1], 0.0; vertices[6].bl << l6[0], l6[1], 0.0;
      // Inside wall 7
      vertices[7].tr << r7[0], r7[1], 2.4; vertices[7].tl << l7[0], l7[1], 2.4;
      vertices[7].br << r7[0], r7[1], 0.0; vertices[7].bl << l7[0], l7[1], 0.0;
      // Inside wall 8
      vertices[8].tr << r8[0], r8[1], 2.4; vertices[8].tl << l8[0], l8[1], 2.4;
      vertices[8].br << r8[0], r8[1], 0.0; vertices[8].bl << l8[0], l8[1], 0.0;
      // Top Wall
      vertices[9].tr << 2.5-t, -10, 2.4; vertices[9].tl << 2.5-t, 10, 2.4;
      vertices[9].br << 2.5-t, -10, 0.0; vertices[9].bl << 2.5-t, 10, 0.0;
      // Bottom Wall
      vertices[10].tr << -2.5+t, 10, 2.4; vertices[10].tl << -2.5+t, -10, 2.4;
      vertices[10].br << -2.5+t, 10, 0.0; vertices[10].bl << -2.5+t, -10, 0.0;
      // Left Wall
      vertices[11].tr << 2.5, 10-t, 2.4; vertices[11].tl << -2.5, 10-t, 2.4;
      vertices[11].br << 2.5, 10-t, 0.0; vertices[11].bl << -2.5, 10-t, 0.0;
      // Floor
      vertices[12].tr << 3.5, -11.0, 0.0; vertices[12].tl << 3.5, 11.0, 0.0;
      vertices[12].br << -3.5,-11.0, 0.0; vertices[12].bl << -3.5, 11.0, 0.0;
      /*// Right Wall
      vertices[13].tr << -1.5, -10+t, 2.4; vertices[3].tl << 2.5, -10-t, 2.4;
      vertices[13].br << -1.5, -10+t, 0.0; vertices[3].bl << 2.5, -10-t, 0.0;*/
    } else if (scene == 5) {
      Eigen::Vector3f Ab, Af, Bb, Bf, Cb, Cf, Db, Df, Eb, Ef, Fb, Ff;
      Eigen::Vector3f Gb, Gf, Hb, Hf, Ib, If, Jb, Jf, Kb, Kf, Lb, Lf;
      Ab << 2.5, -0.7067, 2.623;
      Af << 2.5, -0.7933, 2.5730;
      Bb << -1, -0.7933, 2.5730;
      Bf << -1, -0.7067, 2.6230;
      Cb << -2, -0.7933, 2.5730;
      Cf << -2, -0.7067, 2.6230;
      Db << -2.5, -0.7933, 2.5730;
      Df << -2.5, -0.7067, 2.6230;
      Eb << -2.5, -0.5433, 2.14;
      Ef << -2.5, -0.4567, 2.19;
      Fb << -2.5, -0.2933, 1.707;
      Ff << -2.5, -0.2067, 1.7570;
      Gb << -2.5, 0.7067, -0.0250;
      Gf << -2.5, 0.7933, 0.0250;
      Hb << -2, 0.7067, -0.025;
      Hf << -2, 0.7933, 0.025;
      Ib << -1, 0.7067, -0.0250;
      If << -1, 0.7933, 0.0250;
      Jb << 2.5, 0.7067, -0.025;
      Jf << 2.5, 0.7933, 0.0250;
      Kb << 2.5, -0.2933, 1.7070;
      Kf << 2.5, -0.2067, 1.7570;
      Lb << 2.5, -0.7933, 2.5730;
      Lf << 2.5, -0.7067, 2.623;
      
      vertices[0].tr = Bf; vertices[0].tl = Af;
      vertices[0].br = If; vertices[0].bl = Jf;
      
      vertices[1].tr = Bb; vertices[1].tl = Ab;
      vertices[1].br = Ib; vertices[1].bl = Jb;
      
      vertices[2].tr = Df; vertices[2].tl = Af;
      vertices[2].br = Ef; vertices[2].bl = Lf;
      
      vertices[3].tr = Db; vertices[3].tl = Ab;
      vertices[3].br = Eb; vertices[3].bl = Lb;
      
      vertices[4].tr = Df; vertices[4].tl = Cf;
      vertices[4].br = Gf; vertices[4].bl = Hf;
      
      vertices[5].tr = Db; vertices[5].tl = Cb;
      vertices[5].br = Gb; vertices[5].bl = Hb;
      
      vertices[6].tr = Ff; vertices[6].tl = Kf;
      vertices[6].br = Gf; vertices[6].bl = Jf;
      
      vertices[7].tr = Fb; vertices[7].tl = Kb;
      vertices[7].br = Gb; vertices[7].bl = Jb;
      
      vertices[8].tr << 50, -50, 0; vertices[8].tl << 50, 50, 0;
      vertices[8].br << -50,-50, 0; vertices[8].bl << -50, 50, 0;
    } else if (scene == 6) { // House scenario
      // Outer Left Wall
      vertices[0].tr << 9.8+r, 7.57, 2.4; vertices[0].tl << 0.55-r, 7.57, 2.4;
      vertices[0].br << 9.8+r, 7.57, 0.0; vertices[0].bl << 0.55-r, 7.57, 0.0;
    	// Outer Top Wall
    	vertices[1].tr << 9.67, -2.5-r, 2.4; vertices[1].tl << 9.67, 7.725+r, 2.4;
    	vertices[1].br << 9.67, -2.5-r, 0.0; vertices[1].bl << 9.67, 7.725+r, 0.0;
			// Outer Right Wall
			vertices[2].tr << 0.475-r, -2.4, 2.4; vertices[2].tl << 9.8+r, -2.4, 2.4;
			vertices[2].br << 0.475-r, -2.4, 0.0; vertices[2].bl << 9.8+r, -2.4, 0.0;
			// Outer Bottom Wall
			vertices[3].tr << 0.63, 7.7+r, 2.4; vertices[3].tl << 0.63, -2.45-r, 2.4;
			vertices[3].br << 0.63, 7.7+r, 0.0; vertices[3].bl << 0.63, -2.45-r, 0.0;
			// Inner Left Vertical Wall, Left Face
			vertices[4].tr << 5.7-r, 3.855, 2.4; vertices[4].tl << 8.25+r, 3.855, 2.4;
			vertices[4].br << 5.7-r, 3.855, 0.0; vertices[4].bl << 8.25+r, 3.885, 0.0;
			// Inner Left Vertical Wall, Right Face
			vertices[5].tr << 8.25+r, 3.7, 2.4; vertices[5].tl << 5.7-r, 3.7, 2.4;
			vertices[5].br << 8.25+r, 3.7, 0.0; vertices[5].bl << 5.7-r, 3.7, 0.0;
			// Inner Left Vertical Wall, Top Face
			vertices[6].tr << 8.25, 3.855, 2.4; vertices[6].tl << 8.25, 3.7, 2.4;
			vertices[6].br << 8.25, 3.855, 0.0; vertices[6].bl << 8.25, 3.7, 0.0;
			// Inner Left Vertical Wall, Bottom Face
			vertices[7].tr << 5.7, 3.7, 2.4; vertices[7].tl << 5.7, 3.855, 2.4;
			vertices[7].br << 5.7, 3.7, 0.0; vertices[7].bl << 5.7, 3.855, 0.0;
			// Inner Right Vertical Wall, Left Face
			vertices[8].tr << 5.05-r, 1.655, 2.4; vertices[8].tl << 8.225+r, 1.655, 2.4;
			vertices[8].br << 5.05-r, 1.655, 0.0; vertices[8].bl << 8.225+r, 1.655, 0.0;
			// Inner Right Vertical Wall, Right Face
			vertices[9].tr << 8.225+r, 1.5, 2.4; vertices[9].tl << 5.05-r, 1.5, 2.4;
			vertices[9].br << 8.225+r, 1.5, 0.0; vertices[9].bl << 5.05-r, 1.5, 0.0;
			// Inner Right Vertical Wall, Top Face
			vertices[10].tr << 8.225, 1.655, 2.4; vertices[10].tl << 8.225, 1.5, 2.4;
			vertices[10].br << 8.225, 1.655, 0.0; vertices[10].bl << 8.225, 1.5, 0.0;
			// Inner Right Vertical Wall, Bottom Face
			vertices[11].tr << 5.05, 1.5, 2.4; vertices[11].tl << 8.05, 1.655, 2.4;
			vertices[11].br << 5.05, 1.5, 0.0; vertices[11].bl << 8.05, 1.655, 0.0;
			// Inner Top Left Horizontal Wall, Top Face
			vertices[12].tr << 6.5, 7.725+r, 2.4; vertices[12].tl << 6.5, 3.725-r, 2.4;
			vertices[12].br << 6.5, 7.725+r, 0.0; vertices[12].bl << 6.5, 3.725-r, 0.0;
			// Inner Top Left Horizontal Wall, Bottom Face
			vertices[13].tr << 6.35, 3.725-r, 2.4; vertices[13].tl << 6.35, 7.725+r, 2.4;
			vertices[13].br << 6.25, 3.725-r, 0.0; vertices[13].bl << 6.35, 7.725+r, 0.0;
			// Inner Bottom Left Horizontal Wall, Top Face
			vertices[14].tr << 4.8, 7.725+r, 2.4; vertices[14].tl << 4.8, 3.725-r, 2.4;
			vertices[14].br << 4.8, 7.725+r, 0.0; vertices[14].bl << 4.8, 3.725-r, 0.0;
			// Inner Bottom Left Horizontal Wall, Bottom Face
			vertices[15].tr << 4.65, 3.725-r, 2.4; vertices[15].tl << 4.65, 7.725+r, 2.5;
			vertices[15].br << 4.65, 3.725-r, 0.0; vertices[15].bl << 4.65, 7.725+r, 0.0;
			// Inner Bottom Left Horizontal Wall, Right Face
			vertices[16].tr << 4.8, 3.725, 2.4; vertices[16].tl << 4.65, 3.725, 2.4;
			vertices[16].br << 4.8, 3.725, 0.0; vertices[16].bl << 4.65, 3.725, 0.0;
			// Inner Top Right Horizontal Wall, Top Face
			vertices[17].tr << 6.7, 1.625+r, 2.4; vertices[17].tl << 6.7, -2.375-r, 2.4;
			vertices[17].br << 6.7, 1.625+r, 0.0; vertices[17].bl << 6.7, -2.375-r, 0.0;
			// Inner Top Right Horizontal Wall, Bottom Face
			vertices[18].tr << 6.5, -2.375-r, 2.4; vertices[18].tl << 6.5, 1.625+r, 2.4;
			vertices[18].br << 6.5, -2.375-r, 0.0; vertices[18].bl << 6.5, 1.625+r, 0.0;
			// Inner Bottom Right Horizontal Wall, Top Face
			vertices[19].tr << 3.5, 1.625+r, 2.4; vertices[19].tl << 3.5, -2.375-r, 2.4;
			vertices[19].br << 3.5, 1.625+r, 0.0; vertices[19].bl << 3.5, -2.375-r, 0.0;
			// Inner Bottom Right Horizontal Wall, Bottom Face
			vertices[20].tr << 3.35, -2.375-r, 2.4; vertices[20].tl << 3.35, 1.625+r, 2.4;
			vertices[20].br << 3.35, -2.375-r, 0.0; vertices[20].bl << 3.35, 1.625+r, 0.0;
			// Inner Bottom Right Horizontal Wall, Left Face
			vertices[21].tr << 3.35, 1.625, 2.4; vertices[21].tl << 3.5, 1.625, 2.4;
			vertices[21].br << 3.35, 1.625, 0.0; vertices[21].bl << 3.5, 1.625, 0.0;
			// Floor
			vertices[22].tr << 10, -2.5, 0.0; vertices[22].tl << 10, 7.75, 0.0;
			vertices[22].br << 0.0, -2.5, 0.0; vertices[22].bl << 0.0, 7.75, 0.0;
			// Ceiling
			vertices[23].tr << 0.0, -2.5, 2.4; vertices[23].tl << 0.0, 7.75, 2.4;
			vertices[23].br << 10, -2.5, 2.4; vertices[23].bl << 10, 7.75, 2.4;
    }	else if (scene == 7) { // House 2 scenario
	    t = 0.0625;
	    double r = 0.25; //r = 0.18;
	    double h = 2.4;
      Eigen::Vector2f A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,AA;
      A << 10,15;  B << 4,15;  C << 0,15;   D << 10,11; E << 6,11; F << 5,11;
      G << 4,11;   H << 6,10;  I << 4,10.4; J << 4,9.6; K << 10,9; L << 6,9;
      M << 4,9.5;  N << 0,9.5; O << 8,5;    P << 6,5;   Q << 4,5;  R << 0,5; 
      S << 4,3.95; T << 10,2;  U << 6,2;    V << 4,2;   W << 0,2;  X << 10,0;
      Y << 6,0;    Z << 4,0;   AA << 0,0;
      
      // With doors
      H << 6, 10.1775; 
      F << 4.82, 11; 
      I << 4, 10.32;
        	
      // Outside floor
	    vertices[0].tr << 100, -100, -2.4; vertices[0].tl << 100, 100, -2.4;
	    vertices[0].br << -100, -100, -2.4; vertices[0].bl << -100, 100, -2.4;
      // Inside floor
      vertices[1].tr << X[0], X[1], 0.0+t; vertices[1].tl << A[0], A[1], 0.0+t;
      vertices[1].br << Z[0], Z[1], 0.0+t; vertices[1].bl << B[0], B[1], 0.0+t;
      vertices[2].tr << V[0], V[1], 0.0+t; vertices[2].tl << B[0], B[1], 0.0+t;
      vertices[2].br << R[0], R[1], 0.0+t; vertices[2].bl << C[0], C[1], 0.0+t;
    	// Ceiling inside house
    	/*vertices[3].tr << A[0], A[1], 2.4-t; vertices[3].tl << X[0], X[1], 2.4-t;
    	vertices[3].br << C[0], C[1], 2.4-t; vertices[3].bl << AA[0], AA[1], 2.4-t;*/
    	vertices[3].tr << A[0], A[1], 1.7; vertices[3].tl << X[0], X[1], 1.7;
    	vertices[3].br << C[0], C[1], 1.7; vertices[3].bl << AA[0], AA[1], 1.7;
			// Ceiling outside house
			vertices[4].tr << X[0]+r, X[1]-r, 2.4+t; vertices[4].tl << A[0]+r, A[1]+r, 2.4+t;
			vertices[4].br << AA[0]-r, AA[1]-r, 2.4+t; vertices[4].bl << C[0]-r, C[1]+r, 2.4+t;
			// Stairwell outside wall
			vertices[5].tr << AA[0]-r, AA[1], 2.4; vertices[5].tl << Z[0], Z[1], 2.4;
			vertices[5].br << AA[0]-r, AA[1], -2.4; vertices[5].bl << Z[0], Z[1], -2.4;
			// Stairwell inside wall
			vertices[6].tr << V[0], V[1], 2.4; vertices[6].tl << W[0]-r, W[1], 2.4;
			vertices[6].br << V[0], V[1], -2.4; vertices[6].bl << W[0]-r, W[1], -2.4;
			// Stairs
			vertices[7].tr << Z[0], Z[1], 0.0+r; vertices[7].tl << V[0], V[1], 0.0+r;
			vertices[7].br << AA[0], AA[1], -2.4; vertices[7].bl << W[0], W[1], -2.4;
			// Bottom floor, outside
			vertices[8].tr << W[0]-t, W[1]-r, 0.0; vertices[8].tl << C[0]-t, C[1]+r, 0;
			vertices[8].br << W[0]-t, W[1]-r, -2.4; vertices[8].bl << C[0]-t, C[1]+r, -2.4;
			vertices[9].tr << C[0]-r, C[1]+t, 0.0; vertices[9].tl << A[0]+r, A[1]+t, 0;
			vertices[9].br << C[0]-r, C[1]+t, -2.4; vertices[9].bl << A[0]+r, A[1]+t, -2.4;
			vertices[10].tr << A[0]+t, A[1]+r, 0.0; vertices[10].tl << X[0]+t, X[1]-r, 0;
			vertices[10].br << A[0]+t, A[1]+r, -2.4; vertices[10].bl << X[0]+t, X[1]-r, -2.4;
			vertices[11].tr << X[0]+r, X[1]-t, 0.0; vertices[11].tl << AA[0]-r, AA[1]-t, 0;
			vertices[11].br << X[0]+r, X[1]-t, -2.4; vertices[11].bl << AA[0]-r, AA[1]-t, -2.4;
			// Top floor outside
			vertices[12].tr << AA[0]-t, AA[1]-r, 2.4; vertices[12].tl << C[0]-t, C[1]+r, 2.4;
			vertices[12].br << AA[0]-t, AA[1]-r, 0.0-r; vertices[12].bl << C[0]-t, C[1]+r, 0.0-r;
			vertices[13].tr << C[0]-r, C[1]+t, 2.4; vertices[13].tl << A[0]+r, A[1]+t, 2.4;
			vertices[13].br << C[0]-r, C[1]+t, 0.0; vertices[13].bl << A[0]+r, A[1]+t, 0.0;
			vertices[14].tr << A[0]+t, A[1]+r, 2.4; vertices[14].tl << X[0]+t, X[1]-r, 2.4;
			vertices[14].br << A[0]+t, A[1]+r, 0.0; vertices[14].bl << X[0]+t, X[1]-r, 0.0;
			vertices[15].tr << X[0]+r, X[1]-t, 2.4; vertices[15].tl << AA[0]-r, AA[1]-t, 2.4;
			vertices[15].br << X[0]+r, X[1]-t, 0.0; vertices[15].bl << AA[0]-r, AA[1]-t, 0.0;
			// Stairs narrow outside
			vertices[16].tr << AA[0], AA[1]-t-r, 0.0; vertices[16].tl << AA[0], AA[1]+t+r, 0.0;
			vertices[16].br << AA[0], AA[1]-t-r, -2.4; vertices[16].bl << AA[0], AA[1]+t+r, -2.4;
			// interior boundary walls
			// W to AA
			vertices[17].tr << W[0]+t, W[1], h; vertices[17].tl << AA[0]+t, AA[1], h;
			// AA to Y
			vertices[18].tr << AA[0], AA[1]+t, h; vertices[18].tl << Y[0], Y[1]+t, h;
			// Y to U
			vertices[19].tr << Y[0]-t, Y[1], h; vertices[19].tl << U[0]-t, U[1]+r, h;
			// U to T
			vertices[20].tr << U[0]-r, U[1]+t, h; vertices[20].tl << T[0], T[1]+t, h;
			// T to K
			vertices[21].tr << T[0]-t, T[1], h; vertices[21].tl << K[0]-t, K[1], h;
			// K to L
			vertices[22].tr << K[0], K[1]-t, h; vertices[22].tl << L[0], L[1]-t, h;
			// L to P
			vertices[23].tr << L[0]+t, L[1], h; vertices[23].tl << P[0]+t, P[1], h;
			// P to O
			vertices[24].tr << P[0], P[1]+t, h; vertices[24].tl << O[0]+r, O[1]+t, h;
			// O face
			//vertices[25].tr << O[0], O[1]+t+r, h; vertices[25].tl << O[0], O[1]-t-r, h;
			vertices[25].tr << O[0], O[1]+t, h; vertices[25].tl << O[0], O[1]-t, h;
			// O to P
			vertices[26].tr << O[0]+r, O[1]-t, h; vertices[26].tl << P[0]-r, P[1]-t, h;
			// P to H
			//vertices[27].tr << P[0]-t, P[1]-r, h; vertices[27].tl << H[0]-t, H[1]+r, h;
			vertices[27].tr << P[0]-t, P[1]-r, h; vertices[27].tl << H[0]-t, H[1], h;
			// H face
			//vertices[28].tr << H[0]-t-r, H[1], h; vertices[28].tl << H[0]+t+r, H[1], h;
			vertices[28].tr << H[0]-t, H[1], h; vertices[28].tl << H[0]+t, H[1], h;
			// H to L
			//vertices[29].tr << H[0]+t, H[1]+r, h; vertices[29].tl << L[0]+t, L[1], h;
			vertices[29].tr << H[0]+t, H[1], h; vertices[29].tl << L[0]+t, L[1], h;
			// L to K
			vertices[30].tr << L[0], L[1]+t, h; vertices[30].tl << K[0], K[1]+t, h;
			// K to D
			vertices[31].tr << K[0]-t, K[1], h; vertices[31].tl << D[0]-t, D[1], h;
			// D to F
			vertices[32].tr << D[0], D[1]-t, h; vertices[32].tl << F[0], F[1]-t, h;
			// F face
			//vertices[33].tr << F[0], F[1]-t-r, h; vertices[33].tl << F[0], F[1]+t+r, h;
			vertices[33].tr << F[0], F[1]-t, h; vertices[33].tl << F[0], F[1]+t, h;
			// F to D
			vertices[34].tr << F[0], F[1]+t, h; vertices[34].tl << D[0], D[1]+t, h;
			// D to A
			vertices[35].tr << D[0]-t, D[1], h; vertices[35].tl << A[0]-t, A[1], h;
			// A to B
			vertices[36].tr << A[0], A[1]-t, h; vertices[36].tl << B[0], B[1]-t, h;
			// B to I
			//vertices[37].tr << B[0]+t, B[1], h; vertices[37].tl << I[0]+t, I[1]-r, h;
			vertices[37].tr << B[0]+t, B[1], h; vertices[37].tl << I[0]+t, I[1], h;
			// I face
			//vertices[38].tr << I[0]+t+r, I[1], h; vertices[38].tl << I[0]-t-r, I[1], h;
			vertices[38].tr << I[0]+t, I[1], h; vertices[38].tl << I[0]-t, I[1], h;
			// I to B
			//vertices[39].tr << I[0]-t, I[1]-r, h; vertices[39].tl << B[0]-t, B[1], h;
			vertices[39].tr << I[0]-t, I[1], h; vertices[39].tl << B[0]-t, B[1], h;
			// B to C
			vertices[40].tr << B[0], B[1]-t, h; vertices[40].tl << C[0], C[1]-t, h;
			// C to N
			vertices[41].tr << C[0]+t, C[1], h; vertices[41].tl << N[0]+t, N[1], h;
			//N to M
			vertices[42].tr << N[0], N[1]+t, h; vertices[42].tl << M[0], M[1]+t, h;
			// M to J
			vertices[43].tr << M[0]-t, M[1], h; vertices[43].tl << J[0]-t, J[1], h;
			// J face
			//vertices[44].tr << J[0]-t-r, J[1], h; vertices[44].tl << J[0]+t+r, J[1], h;
			vertices[44].tr << J[0]-t, J[1], h; vertices[44].tl << J[0]+t, J[1], h;
			// J to M
			//vertices[45].tr << J[0]+t, J[1]+r, h; vertices[45].tl << M[0]+t, M[1]-r, h;
			vertices[45].tr << J[0]+t, J[1], h; vertices[45].tl << M[0]+t, M[1], h;
			// M to N
			vertices[46].tr << M[0]+r, M[1]-t, h; vertices[46].tl << N[0], N[1]-t, h;
			// N to R
			vertices[47].tr << N[0]+t, N[1], h; vertices[47].tl << R[0]+t, R[1], h; 
			// R to Q
			vertices[48].tr << R[0], R[1]+t, h; vertices[48].tl << Q[0]+r, Q[1]+t, h;
			// Q to V
			vertices[49].tr << Q[0]+t, Q[1]+r, h; vertices[49].tl << V[0]+t, V[1]-r, h;			
			// V to W is already done
			// W to AA is the beginning
			// Bottom of upstairs by the stairs
			vertices[50].tr << AA[0]-t, AA[1]-r, 0.0; vertices[50].tl << W[0]-t, W[1]+r, 0.0;
			vertices[50].br << AA[0]+t, AA[1]-r, 0.0; vertices[50].bl << W[0]+t, W[1]+r, 0.0;
			
			for (int i = 17; i < 50; i++)	{
				vertices[i].br << vertices[i].tr[0], vertices[i].tr[1], 0.0;
				vertices[i].bl << vertices[i].tl[0], vertices[i].tl[1], 0.0;
			}
    }
  } else {
    t = 2.44;
    // Wall 0
    vertices[0].tr << 0.0, 0.0, t;   vertices[0].tl << 0.0, -t, t;
    vertices[0].br << 0.0, 0.0, 0.0; vertices[0].bl << 0.0, -t, 0.0;
    normals[0] << 1.0, 0.0, 0.0;
    // Wall 1
    vertices[1].tr << t, 0.0, t;   vertices[1].tl << 0.0, 0.0, t;
    vertices[1].br << t, 0.0, 0.0; vertices[1].bl << 0.0, 0.0, 0.0;
    normals[1] << 0.0, -1.0, 0.0;
    // Wall 2
    vertices[2].tr << t, -t, t;   vertices[2].tl << t, 0.0, t;
    vertices[2].br << t, -t, 0.0; vertices[2].bl << t, 0.0, 0.0;
    normals[2] << -1.0, 0.0, 0.0;
    // Wall 3
    vertices[3].tr << 0.0, -t, t;   vertices[3].tl << t, -t, t;
    vertices[3].br << 0.0, -t, 0.0; vertices[3].bl << t, -t, 0.0;
    normals[3] << 0.0, 1.0, 0.0;
    // Floor
    vertices[4].tr << 10.0, -10.0, 0.0; vertices[4].tl << 10.0, 0.0,  0.0;
    vertices[4].br << 0.0,  -10.0, 0.0; vertices[4].bl << 0.0,  0.0,  0.0;
    normals[4] << 0.0,   0.0,  1.0;
    // Ceiling
    vertices[5].tr << 0.0,  -10.0, t; vertices[5].tl << 0.0,  0.0,  t;
    vertices[5].br << 10.0, -10.0, t; vertices[5].bl << 10.0, 0.0,  t;
    normals[5] << 0.0, 0.0, -1.0;
  }
}   
