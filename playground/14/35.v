// Benchmark "adder" written by ABC on Wed Jul 17 19:27:00 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n96, new_n97, new_n98, new_n99, new_n100, new_n101, new_n102,
    new_n103, new_n104, new_n105, new_n106, new_n107, new_n108, new_n109,
    new_n110, new_n111, new_n112, new_n113, new_n114, new_n115, new_n116,
    new_n117, new_n118, new_n119, new_n120, new_n121, new_n122, new_n123,
    new_n124, new_n125, new_n126, new_n127, new_n128, new_n129, new_n130,
    new_n131, new_n132, new_n133, new_n134, new_n135, new_n136, new_n137,
    new_n138, new_n139, new_n140, new_n141, new_n142, new_n143, new_n144,
    new_n145, new_n146, new_n148, new_n149, new_n150, new_n151, new_n152,
    new_n153, new_n154, new_n155, new_n157, new_n158, new_n159, new_n160,
    new_n161, new_n162, new_n164, new_n165, new_n166, new_n167, new_n168,
    new_n169, new_n170, new_n171, new_n172, new_n173, new_n174, new_n175,
    new_n176, new_n177, new_n179, new_n180, new_n181, new_n182, new_n183,
    new_n184, new_n186, new_n187, new_n188, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n197, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n204, new_n205, new_n206, new_n207,
    new_n208, new_n209, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n241, new_n242, new_n243, new_n244, new_n245, new_n246,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n283, new_n284, new_n285,
    new_n286, new_n287, new_n288, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n314, new_n315, new_n316,
    new_n317, new_n318, new_n319, new_n320, new_n321, new_n322, new_n323,
    new_n324, new_n325, new_n326, new_n328, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n335, new_n336, new_n337, new_n338, new_n339,
    new_n340, new_n341, new_n342, new_n343, new_n344, new_n346, new_n347,
    new_n348, new_n350, new_n351, new_n352, new_n353, new_n354, new_n355,
    new_n356, new_n357, new_n358, new_n359, new_n361, new_n362, new_n363,
    new_n364, new_n365, new_n366, new_n367, new_n368, new_n369, new_n370,
    new_n372, new_n374, new_n375, new_n377, new_n378, new_n379, new_n380,
    new_n382, new_n383, new_n384, new_n386, new_n387, new_n388, new_n389,
    new_n391, new_n392, new_n393, new_n395;
  assign new_n96 = ~\a[9]  & ~\b[8] ;
  assign new_n97 = ~\a[2]  & ~\b[1] ;
  assign new_n98 = \a[2]  & \b[1] ;
  assign new_n99 = \a[1]  & \b[0] ;
  assign new_n100 = ~new_n98 & ~new_n99;
  assign new_n101 = ~new_n97 & ~new_n100;
  assign new_n102 = ~\a[3]  & ~\b[2] ;
  assign new_n103 = \a[3]  & \b[2] ;
  assign new_n104 = ~new_n102 & ~new_n103;
  assign new_n105 = ~new_n101 & new_n104;
  assign new_n106 = \a[4]  & \b[3] ;
  assign new_n107 = ~\a[4]  & ~\b[3] ;
  assign new_n108 = ~new_n106 & ~new_n107;
  assign new_n109 = ~new_n102 & new_n108;
  assign new_n110 = ~new_n105 & new_n109;
  assign new_n111 = ~\a[7]  & ~\b[6] ;
  assign new_n112 = ~\a[5]  & ~\b[4] ;
  assign new_n113 = ~\a[6]  & ~\b[5] ;
  assign new_n114 = ~new_n112 & ~new_n113;
  assign new_n115 = ~new_n111 & new_n114;
  assign new_n116 = ~\a[8]  & ~\b[7] ;
  assign new_n117 = ~new_n106 & ~new_n116;
  assign new_n118 = \a[6]  & \b[5] ;
  assign new_n119 = \a[7]  & \b[6] ;
  assign new_n120 = ~new_n118 & ~new_n119;
  assign new_n121 = \a[8]  & \b[7] ;
  assign new_n122 = \a[5]  & \b[4] ;
  assign new_n123 = ~new_n121 & ~new_n122;
  assign new_n124 = new_n120 & new_n123;
  assign new_n125 = new_n117 & new_n124;
  assign new_n126 = new_n115 & new_n125;
  assign new_n127 = ~new_n110 & new_n126;
  assign new_n128 = ~new_n111 & ~new_n116;
  assign new_n129 = ~new_n121 & ~new_n128;
  assign new_n130 = ~new_n113 & ~new_n118;
  assign new_n131 = ~new_n112 & new_n130;
  assign new_n132 = ~new_n111 & new_n120;
  assign new_n133 = ~new_n116 & ~new_n121;
  assign new_n134 = new_n132 & new_n133;
  assign new_n135 = ~new_n131 & new_n134;
  assign new_n136 = ~new_n129 & ~new_n135;
  assign new_n137 = ~new_n127 & new_n136;
  assign new_n138 = \a[9]  & \b[8] ;
  assign new_n139 = ~new_n96 & ~new_n138;
  assign new_n140 = ~new_n137 & new_n139;
  assign new_n141 = ~new_n96 & ~new_n140;
  assign new_n142 = ~\a[10]  & ~\b[9] ;
  assign new_n143 = \a[10]  & \b[9] ;
  assign new_n144 = ~new_n142 & ~new_n143;
  assign new_n145 = ~new_n141 & new_n144;
  assign new_n146 = new_n141 & ~new_n144;
  assign \s[10]  = ~new_n145 & ~new_n146;
  assign new_n148 = ~new_n96 & new_n144;
  assign new_n149 = ~new_n143 & ~new_n148;
  assign new_n150 = ~new_n145 & ~new_n149;
  assign new_n151 = ~\a[11]  & ~\b[10] ;
  assign new_n152 = \a[11]  & \b[10] ;
  assign new_n153 = ~new_n151 & ~new_n152;
  assign new_n154 = ~new_n150 & new_n153;
  assign new_n155 = new_n150 & ~new_n153;
  assign \s[11]  = ~new_n154 & ~new_n155;
  assign new_n157 = ~new_n151 & ~new_n154;
  assign new_n158 = ~\a[12]  & ~\b[11] ;
  assign new_n159 = \a[12]  & \b[11] ;
  assign new_n160 = ~new_n158 & ~new_n159;
  assign new_n161 = ~new_n157 & new_n160;
  assign new_n162 = new_n157 & ~new_n160;
  assign \s[12]  = ~new_n161 & ~new_n162;
  assign new_n164 = new_n153 & new_n160;
  assign new_n165 = new_n139 & new_n144;
  assign new_n166 = new_n164 & new_n165;
  assign new_n167 = ~new_n137 & new_n166;
  assign new_n168 = new_n151 & ~new_n159;
  assign new_n169 = ~new_n158 & ~new_n168;
  assign new_n170 = new_n149 & new_n164;
  assign new_n171 = new_n169 & ~new_n170;
  assign new_n172 = ~new_n167 & new_n171;
  assign new_n173 = ~\a[13]  & ~\b[12] ;
  assign new_n174 = \a[13]  & \b[12] ;
  assign new_n175 = ~new_n173 & ~new_n174;
  assign new_n176 = ~new_n172 & new_n175;
  assign new_n177 = new_n172 & ~new_n175;
  assign \s[13]  = ~new_n176 & ~new_n177;
  assign new_n179 = ~new_n173 & ~new_n176;
  assign new_n180 = ~\a[14]  & ~\b[13] ;
  assign new_n181 = \a[14]  & \b[13] ;
  assign new_n182 = ~new_n180 & ~new_n181;
  assign new_n183 = ~new_n179 & new_n182;
  assign new_n184 = new_n179 & ~new_n182;
  assign \s[14]  = ~new_n183 & ~new_n184;
  assign new_n186 = new_n175 & new_n182;
  assign new_n187 = ~new_n172 & new_n186;
  assign new_n188 = ~new_n173 & ~new_n180;
  assign new_n189 = ~new_n181 & ~new_n188;
  assign new_n190 = ~new_n187 & ~new_n189;
  assign new_n191 = ~\a[15]  & ~\b[14] ;
  assign new_n192 = \a[15]  & \b[14] ;
  assign new_n193 = ~new_n191 & ~new_n192;
  assign new_n194 = ~new_n190 & new_n193;
  assign new_n195 = new_n190 & ~new_n193;
  assign \s[15]  = ~new_n194 & ~new_n195;
  assign new_n197 = ~new_n191 & ~new_n194;
  assign new_n198 = ~\a[16]  & ~\b[15] ;
  assign new_n199 = \a[16]  & \b[15] ;
  assign new_n200 = ~new_n198 & ~new_n199;
  assign new_n201 = ~new_n197 & new_n200;
  assign new_n202 = new_n197 & ~new_n200;
  assign \s[16]  = ~new_n201 & ~new_n202;
  assign new_n204 = new_n193 & new_n200;
  assign new_n205 = new_n186 & new_n204;
  assign new_n206 = new_n166 & new_n205;
  assign new_n207 = ~new_n137 & new_n206;
  assign new_n208 = ~new_n171 & new_n205;
  assign new_n209 = new_n189 & ~new_n192;
  assign new_n210 = ~new_n191 & ~new_n209;
  assign new_n211 = ~new_n199 & ~new_n210;
  assign new_n212 = ~new_n198 & ~new_n211;
  assign new_n213 = ~new_n208 & new_n212;
  assign new_n214 = ~new_n207 & new_n213;
  assign new_n215 = ~\a[17]  & ~\b[16] ;
  assign new_n216 = \a[17]  & \b[16] ;
  assign new_n217 = ~new_n215 & ~new_n216;
  assign new_n218 = ~new_n214 & new_n217;
  assign new_n219 = new_n214 & ~new_n217;
  assign \s[17]  = ~new_n218 & ~new_n219;
  assign new_n221 = ~new_n215 & ~new_n218;
  assign new_n222 = ~\a[18]  & ~\b[17] ;
  assign new_n223 = \a[18]  & \b[17] ;
  assign new_n224 = ~new_n222 & ~new_n223;
  assign new_n225 = ~new_n221 & new_n224;
  assign new_n226 = new_n221 & ~new_n224;
  assign \s[18]  = ~new_n225 & ~new_n226;
  assign new_n228 = new_n217 & new_n224;
  assign new_n229 = ~new_n214 & new_n228;
  assign new_n230 = ~new_n215 & ~new_n222;
  assign new_n231 = ~new_n223 & ~new_n230;
  assign new_n232 = ~new_n229 & ~new_n231;
  assign new_n233 = ~\a[19]  & ~\b[18] ;
  assign new_n234 = \a[19]  & \b[18] ;
  assign new_n235 = ~new_n233 & ~new_n234;
  assign new_n236 = ~new_n232 & new_n235;
  assign new_n237 = new_n232 & ~new_n235;
  assign \s[19]  = ~new_n236 & ~new_n237;
  assign new_n239 = ~\a[1]  & ~\b[0] ;
  assign \s[1]  = new_n99 | new_n239;
  assign new_n241 = ~new_n233 & ~new_n236;
  assign new_n242 = ~\a[20]  & ~\b[19] ;
  assign new_n243 = \a[20]  & \b[19] ;
  assign new_n244 = ~new_n242 & ~new_n243;
  assign new_n245 = ~new_n241 & new_n244;
  assign new_n246 = new_n241 & ~new_n244;
  assign \s[20]  = ~new_n245 & ~new_n246;
  assign new_n248 = new_n235 & new_n244;
  assign new_n249 = new_n228 & new_n248;
  assign new_n250 = ~new_n214 & new_n249;
  assign new_n251 = new_n233 & ~new_n243;
  assign new_n252 = ~new_n242 & ~new_n251;
  assign new_n253 = new_n231 & new_n248;
  assign new_n254 = new_n252 & ~new_n253;
  assign new_n255 = ~new_n250 & new_n254;
  assign new_n256 = ~\a[21]  & ~\b[20] ;
  assign new_n257 = \a[21]  & \b[20] ;
  assign new_n258 = ~new_n256 & ~new_n257;
  assign new_n259 = ~new_n255 & new_n258;
  assign new_n260 = new_n255 & ~new_n258;
  assign \s[21]  = ~new_n259 & ~new_n260;
  assign new_n262 = ~new_n256 & ~new_n259;
  assign new_n263 = ~\a[22]  & ~\b[21] ;
  assign new_n264 = \a[22]  & \b[21] ;
  assign new_n265 = ~new_n263 & ~new_n264;
  assign new_n266 = ~new_n262 & new_n265;
  assign new_n267 = new_n262 & ~new_n265;
  assign \s[22]  = ~new_n266 & ~new_n267;
  assign new_n269 = new_n258 & new_n265;
  assign new_n270 = new_n249 & new_n269;
  assign new_n271 = ~new_n214 & new_n270;
  assign new_n272 = ~new_n254 & new_n269;
  assign new_n273 = new_n256 & ~new_n264;
  assign new_n274 = ~new_n263 & ~new_n273;
  assign new_n275 = ~new_n272 & new_n274;
  assign new_n276 = ~new_n271 & new_n275;
  assign new_n277 = ~\a[23]  & ~\b[22] ;
  assign new_n278 = \a[23]  & \b[22] ;
  assign new_n279 = ~new_n277 & ~new_n278;
  assign new_n280 = ~new_n276 & new_n279;
  assign new_n281 = new_n276 & ~new_n279;
  assign \s[23]  = ~new_n280 & ~new_n281;
  assign new_n283 = ~new_n277 & ~new_n280;
  assign new_n284 = ~\a[24]  & ~\b[23] ;
  assign new_n285 = \a[24]  & \b[23] ;
  assign new_n286 = ~new_n284 & ~new_n285;
  assign new_n287 = ~new_n283 & new_n286;
  assign new_n288 = new_n283 & ~new_n286;
  assign \s[24]  = ~new_n287 & ~new_n288;
  assign new_n290 = new_n279 & new_n286;
  assign new_n291 = new_n269 & new_n290;
  assign new_n292 = new_n249 & new_n291;
  assign new_n293 = ~new_n214 & new_n292;
  assign new_n294 = ~new_n254 & new_n291;
  assign new_n295 = ~new_n274 & ~new_n278;
  assign new_n296 = ~new_n277 & ~new_n295;
  assign new_n297 = ~new_n285 & ~new_n296;
  assign new_n298 = ~new_n284 & ~new_n297;
  assign new_n299 = ~new_n294 & new_n298;
  assign new_n300 = ~new_n293 & new_n299;
  assign new_n301 = ~\a[25]  & ~\b[24] ;
  assign new_n302 = \a[25]  & \b[24] ;
  assign new_n303 = ~new_n301 & ~new_n302;
  assign new_n304 = ~new_n300 & new_n303;
  assign new_n305 = new_n300 & ~new_n303;
  assign \s[25]  = ~new_n304 & ~new_n305;
  assign new_n307 = ~new_n301 & ~new_n304;
  assign new_n308 = ~\a[26]  & ~\b[25] ;
  assign new_n309 = \a[26]  & \b[25] ;
  assign new_n310 = ~new_n308 & ~new_n309;
  assign new_n311 = ~new_n307 & new_n310;
  assign new_n312 = new_n307 & ~new_n310;
  assign \s[26]  = ~new_n311 & ~new_n312;
  assign new_n314 = new_n303 & new_n310;
  assign new_n315 = new_n292 & new_n314;
  assign new_n316 = ~new_n214 & new_n315;
  assign new_n317 = ~new_n299 & new_n314;
  assign new_n318 = new_n301 & ~new_n309;
  assign new_n319 = ~new_n308 & ~new_n318;
  assign new_n320 = ~new_n317 & new_n319;
  assign new_n321 = ~new_n316 & new_n320;
  assign new_n322 = ~\a[27]  & ~\b[26] ;
  assign new_n323 = \a[27]  & \b[26] ;
  assign new_n324 = ~new_n322 & ~new_n323;
  assign new_n325 = ~new_n321 & new_n324;
  assign new_n326 = new_n321 & ~new_n324;
  assign \s[27]  = ~new_n325 & ~new_n326;
  assign new_n328 = ~new_n322 & ~new_n325;
  assign new_n329 = ~\a[28]  & ~\b[27] ;
  assign new_n330 = \a[28]  & \b[27] ;
  assign new_n331 = ~new_n329 & ~new_n330;
  assign new_n332 = ~new_n328 & new_n331;
  assign new_n333 = new_n328 & ~new_n331;
  assign \s[28]  = ~new_n332 & ~new_n333;
  assign new_n335 = new_n324 & new_n331;
  assign new_n336 = ~new_n321 & new_n335;
  assign new_n337 = ~new_n322 & ~new_n329;
  assign new_n338 = ~new_n330 & ~new_n337;
  assign new_n339 = ~new_n336 & ~new_n338;
  assign new_n340 = ~\a[29]  & ~\b[28] ;
  assign new_n341 = \a[29]  & \b[28] ;
  assign new_n342 = ~new_n340 & ~new_n341;
  assign new_n343 = ~new_n339 & new_n342;
  assign new_n344 = new_n339 & ~new_n342;
  assign \s[29]  = ~new_n343 & ~new_n344;
  assign new_n346 = ~new_n97 & ~new_n98;
  assign new_n347 = new_n99 & ~new_n346;
  assign new_n348 = ~new_n99 & new_n346;
  assign \s[2]  = ~new_n347 & ~new_n348;
  assign new_n350 = new_n335 & new_n342;
  assign new_n351 = ~new_n321 & new_n350;
  assign new_n352 = ~new_n338 & ~new_n340;
  assign new_n353 = ~new_n341 & ~new_n352;
  assign new_n354 = ~new_n351 & ~new_n353;
  assign new_n355 = ~\a[30]  & ~\b[29] ;
  assign new_n356 = \a[30]  & \b[29] ;
  assign new_n357 = ~new_n355 & ~new_n356;
  assign new_n358 = ~new_n354 & new_n357;
  assign new_n359 = new_n354 & ~new_n357;
  assign \s[30]  = ~new_n358 & ~new_n359;
  assign new_n361 = \a[31]  & ~\b[30] ;
  assign new_n362 = ~\a[31]  & \b[30] ;
  assign new_n363 = ~new_n361 & ~new_n362;
  assign new_n364 = new_n350 & new_n357;
  assign new_n365 = ~new_n321 & new_n364;
  assign new_n366 = new_n353 & ~new_n356;
  assign new_n367 = ~new_n355 & ~new_n366;
  assign new_n368 = ~new_n365 & new_n367;
  assign new_n369 = ~new_n363 & ~new_n368;
  assign new_n370 = new_n363 & new_n368;
  assign \s[31]  = ~new_n369 & ~new_n370;
  assign new_n372 = new_n101 & ~new_n104;
  assign \s[3]  = ~new_n105 & ~new_n372;
  assign new_n374 = ~new_n102 & ~new_n105;
  assign new_n375 = ~new_n108 & ~new_n374;
  assign \s[4]  = new_n110 | new_n375;
  assign new_n377 = ~new_n112 & ~new_n122;
  assign new_n378 = ~new_n106 & ~new_n110;
  assign new_n379 = ~new_n377 & ~new_n378;
  assign new_n380 = new_n377 & new_n378;
  assign \s[5]  = ~new_n379 & ~new_n380;
  assign new_n382 = ~new_n112 & ~new_n380;
  assign new_n383 = ~new_n130 & ~new_n382;
  assign new_n384 = new_n131 & ~new_n380;
  assign \s[6]  = new_n383 | new_n384;
  assign new_n386 = ~new_n111 & ~new_n119;
  assign new_n387 = ~new_n118 & ~new_n384;
  assign new_n388 = ~new_n386 & ~new_n387;
  assign new_n389 = new_n132 & ~new_n384;
  assign \s[7]  = ~new_n388 & ~new_n389;
  assign new_n391 = ~new_n111 & ~new_n389;
  assign new_n392 = new_n133 & ~new_n391;
  assign new_n393 = ~new_n133 & new_n391;
  assign \s[8]  = ~new_n392 & ~new_n393;
  assign new_n395 = new_n137 & ~new_n139;
  assign \s[9]  = ~new_n140 & ~new_n395;
  assign \s[0]  = ~\a[0] ;
endmodule


