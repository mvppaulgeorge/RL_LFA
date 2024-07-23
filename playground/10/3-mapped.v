// Benchmark "adder" written by ABC on Wed Jul 17 17:03:37 2024

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
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n141, new_n142, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n151, new_n152, new_n153,
    new_n155, new_n156, new_n157, new_n158, new_n159, new_n160, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n191, new_n192,
    new_n193, new_n194, new_n195, new_n196, new_n197, new_n198, new_n200,
    new_n201, new_n202, new_n203, new_n204, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n242, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n282, new_n283, new_n284, new_n285,
    new_n286, new_n287, new_n288, new_n289, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n321, new_n322, new_n323,
    new_n324, new_n325, new_n326, new_n327, new_n329, new_n330, new_n331,
    new_n332, new_n333, new_n334, new_n335, new_n336, new_n337, new_n338,
    new_n339, new_n340, new_n341, new_n343, new_n344, new_n345, new_n346,
    new_n347, new_n348, new_n349, new_n350, new_n351, new_n353, new_n354,
    new_n355, new_n356, new_n357, new_n358, new_n359, new_n360, new_n361,
    new_n364, new_n365, new_n366, new_n367, new_n368, new_n369, new_n370,
    new_n371, new_n372, new_n373, new_n374, new_n375, new_n377, new_n378,
    new_n379, new_n380, new_n381, new_n382, new_n383, new_n384, new_n387,
    new_n389, new_n390, new_n391, new_n392, new_n393, new_n395, new_n397,
    new_n398, new_n399, new_n400, new_n402, new_n403, new_n404, new_n405,
    new_n407, new_n408;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n24x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand42aa1n16x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nona22aa1n09x5               g005(.a(new_n99), .b(new_n98), .c(new_n100), .out0(new_n101));
  nor042aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1d28x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor042aa1d18x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano22aa1n06x5               g011(.a(new_n105), .b(new_n99), .c(new_n106), .out0(new_n107));
  nand03aa1n04x5               g012(.a(new_n107), .b(new_n101), .c(new_n104), .o1(new_n108));
  aoi012aa1n06x5               g013(.a(new_n102), .b(new_n105), .c(new_n103), .o1(new_n109));
  nor002aa1n20x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d28x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor042aa1d18x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n24x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1d15x5               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  nor002aa1d32x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  tech160nm_fiaoi012aa1n04x5   g020(.a(new_n115), .b(\a[6] ), .c(\b[5] ), .o1(new_n116));
  inv040aa1d32x5               g021(.a(\a[6] ), .o1(new_n117));
  inv040aa1d32x5               g022(.a(\b[5] ), .o1(new_n118));
  aoi022aa1n06x5               g023(.a(new_n118), .b(new_n117), .c(\a[5] ), .d(\b[4] ), .o1(new_n119));
  nand23aa1n06x5               g024(.a(new_n114), .b(new_n116), .c(new_n119), .o1(new_n120));
  oao003aa1n09x5               g025(.a(new_n117), .b(new_n118), .c(new_n115), .carry(new_n121));
  ao0012aa1n03x5               g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  aoi012aa1d24x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n120), .c(new_n108), .d(new_n109), .o1(new_n124));
  xnrc02aa1n12x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  tech160nm_fiaoi012aa1n05x5   g031(.a(new_n97), .b(new_n124), .c(new_n126), .o1(new_n127));
  inv000aa1d42x5               g032(.a(\a[10] ), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\b[9] ), .o1(new_n129));
  nanp02aa1n06x5               g034(.a(new_n129), .b(new_n128), .o1(new_n130));
  nand02aa1d08x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\a[4] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[3] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  nanb02aa1n06x5               g039(.a(new_n105), .b(new_n106), .out0(new_n135));
  nano32aa1n06x5               g040(.a(new_n135), .b(new_n134), .c(new_n103), .d(new_n99), .out0(new_n136));
  inv020aa1n02x5               g041(.a(new_n109), .o1(new_n137));
  nona23aa1n03x5               g042(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n138));
  nano22aa1n03x7               g043(.a(new_n138), .b(new_n116), .c(new_n119), .out0(new_n139));
  aoai13aa1n06x5               g044(.a(new_n139), .b(new_n137), .c(new_n136), .d(new_n101), .o1(new_n140));
  nano22aa1n02x4               g045(.a(new_n97), .b(new_n130), .c(new_n131), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n125), .c(new_n140), .d(new_n123), .o1(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n127), .c(new_n131), .d(new_n130), .o1(\s[10] ));
  nano22aa1d15x5               g048(.a(new_n125), .b(new_n130), .c(new_n131), .out0(new_n144));
  aobi12aa1n02x5               g049(.a(new_n144), .b(new_n140), .c(new_n123), .out0(new_n145));
  nanp03aa1n02x5               g050(.a(new_n130), .b(new_n97), .c(new_n131), .o1(new_n146));
  nand02aa1n06x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norp02aa1n12x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n148), .b(new_n128), .c(new_n129), .o1(new_n149));
  nanp03aa1n02x5               g054(.a(new_n146), .b(new_n147), .c(new_n149), .o1(new_n150));
  aob012aa1n03x5               g055(.a(new_n130), .b(new_n97), .c(new_n131), .out0(new_n151));
  aoi012aa1n02x5               g056(.a(new_n151), .b(new_n124), .c(new_n144), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n147), .b(new_n148), .out0(new_n153));
  oai022aa1n02x5               g058(.a(new_n152), .b(new_n153), .c(new_n150), .d(new_n145), .o1(\s[11] ));
  oaoi13aa1n02x5               g059(.a(new_n148), .b(new_n147), .c(new_n145), .d(new_n151), .o1(new_n155));
  nor022aa1n16x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  nand22aa1n09x5               g061(.a(\b[11] ), .b(\a[12] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  aoai13aa1n03x5               g063(.a(new_n153), .b(new_n151), .c(new_n124), .d(new_n144), .o1(new_n159));
  nona23aa1n03x5               g064(.a(new_n159), .b(new_n157), .c(new_n156), .d(new_n148), .out0(new_n160));
  oai012aa1n03x5               g065(.a(new_n160), .b(new_n155), .c(new_n158), .o1(\s[12] ));
  nano23aa1n06x5               g066(.a(new_n156), .b(new_n148), .c(new_n157), .d(new_n147), .out0(new_n162));
  nanp02aa1n02x5               g067(.a(new_n144), .b(new_n162), .o1(new_n163));
  nano22aa1n02x4               g068(.a(new_n156), .b(new_n148), .c(new_n157), .out0(new_n164));
  nanp02aa1n04x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nor002aa1d24x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  oai112aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(\b[11] ), .d(\a[12] ), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n164), .b(new_n168), .c(new_n162), .d(new_n151), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n163), .c(new_n140), .d(new_n123), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n156), .b(new_n148), .c(new_n157), .o1(new_n171));
  aobi12aa1n06x5               g076(.a(new_n171), .b(new_n162), .c(new_n151), .out0(new_n172));
  aoai13aa1n04x5               g077(.a(new_n172), .b(new_n163), .c(new_n140), .d(new_n123), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n166), .b(new_n165), .out0(new_n174));
  aob012aa1n02x5               g079(.a(new_n170), .b(new_n173), .c(new_n174), .out0(\s[13] ));
  nanb02aa1n02x5               g080(.a(new_n174), .b(new_n173), .out0(new_n176));
  nor002aa1n06x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nand22aa1n06x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  nona23aa1n02x4               g084(.a(new_n176), .b(new_n178), .c(new_n177), .d(new_n166), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n176), .d(new_n167), .o1(\s[14] ));
  nano23aa1n03x7               g086(.a(new_n177), .b(new_n166), .c(new_n178), .d(new_n165), .out0(new_n182));
  and003aa1n02x5               g087(.a(new_n144), .b(new_n182), .c(new_n162), .o(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n128), .b(new_n129), .c(new_n97), .o1(new_n184));
  nona23aa1n09x5               g089(.a(new_n147), .b(new_n157), .c(new_n156), .d(new_n148), .out0(new_n185));
  oai012aa1n03x5               g090(.a(new_n171), .b(new_n185), .c(new_n184), .o1(new_n186));
  nano22aa1n02x4               g091(.a(new_n177), .b(new_n166), .c(new_n178), .out0(new_n187));
  nand02aa1n03x5               g092(.a(\b[14] ), .b(\a[15] ), .o1(new_n188));
  nor002aa1d32x5               g093(.a(\b[14] ), .b(\a[15] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n189), .o1(new_n190));
  oai112aa1n02x5               g095(.a(new_n190), .b(new_n188), .c(\b[13] ), .d(\a[14] ), .o1(new_n191));
  aoi112aa1n02x5               g096(.a(new_n191), .b(new_n187), .c(new_n186), .d(new_n182), .o1(new_n192));
  aob012aa1n02x5               g097(.a(new_n192), .b(new_n124), .c(new_n183), .out0(new_n193));
  nona23aa1n03x5               g098(.a(new_n165), .b(new_n178), .c(new_n177), .d(new_n166), .out0(new_n194));
  aoi012aa1n02x7               g099(.a(new_n177), .b(new_n166), .c(new_n178), .o1(new_n195));
  oai012aa1n02x5               g100(.a(new_n195), .b(new_n172), .c(new_n194), .o1(new_n196));
  aoi012aa1n02x5               g101(.a(new_n196), .b(new_n124), .c(new_n183), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n188), .b(new_n189), .out0(new_n198));
  oai012aa1n02x5               g103(.a(new_n193), .b(new_n197), .c(new_n198), .o1(\s[15] ));
  aoai13aa1n04x5               g104(.a(new_n198), .b(new_n196), .c(new_n124), .d(new_n183), .o1(new_n200));
  nor042aa1n04x5               g105(.a(\b[15] ), .b(\a[16] ), .o1(new_n201));
  nand02aa1d06x5               g106(.a(\b[15] ), .b(\a[16] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona23aa1n03x5               g108(.a(new_n200), .b(new_n202), .c(new_n201), .d(new_n189), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n204), .b(new_n203), .c(new_n190), .d(new_n200), .o1(\s[16] ));
  nona23aa1n09x5               g110(.a(new_n188), .b(new_n202), .c(new_n201), .d(new_n189), .out0(new_n206));
  nona23aa1n08x5               g111(.a(new_n182), .b(new_n144), .c(new_n206), .d(new_n185), .out0(new_n207));
  nor042aa1n02x5               g112(.a(new_n206), .b(new_n194), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n201), .b(new_n189), .c(new_n202), .o1(new_n209));
  oai012aa1n02x5               g114(.a(new_n209), .b(new_n206), .c(new_n195), .o1(new_n210));
  aoi012aa1n06x5               g115(.a(new_n210), .b(new_n186), .c(new_n208), .o1(new_n211));
  aoai13aa1n04x5               g116(.a(new_n211), .b(new_n207), .c(new_n140), .d(new_n123), .o1(new_n212));
  nor002aa1d32x5               g117(.a(\b[16] ), .b(\a[17] ), .o1(new_n213));
  nand42aa1n04x5               g118(.a(\b[16] ), .b(\a[17] ), .o1(new_n214));
  norb02aa1n06x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  inv040aa1n04x5               g120(.a(new_n207), .o1(new_n216));
  nanb03aa1n02x5               g121(.a(new_n201), .b(new_n202), .c(new_n189), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n213), .o1(new_n218));
  nano32aa1n02x4               g123(.a(new_n201), .b(new_n214), .c(new_n217), .d(new_n218), .out0(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n206), .c(new_n195), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n220), .b(new_n186), .c(new_n208), .o1(new_n221));
  aob012aa1n02x5               g126(.a(new_n221), .b(new_n124), .c(new_n216), .out0(new_n222));
  oaib12aa1n02x5               g127(.a(new_n222), .b(new_n215), .c(new_n212), .out0(\s[17] ));
  oa0012aa1n02x5               g128(.a(new_n209), .b(new_n206), .c(new_n195), .o(new_n224));
  oaib12aa1n12x5               g129(.a(new_n224), .b(new_n172), .c(new_n208), .out0(new_n225));
  aoai13aa1n04x5               g130(.a(new_n215), .b(new_n225), .c(new_n124), .d(new_n216), .o1(new_n226));
  nor002aa1d32x5               g131(.a(\b[17] ), .b(\a[18] ), .o1(new_n227));
  nand02aa1d28x5               g132(.a(\b[17] ), .b(\a[18] ), .o1(new_n228));
  norb02aa1n03x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  nona23aa1n03x5               g134(.a(new_n226), .b(new_n228), .c(new_n227), .d(new_n213), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n229), .c(new_n218), .d(new_n226), .o1(\s[18] ));
  nano23aa1n03x7               g136(.a(new_n213), .b(new_n227), .c(new_n228), .d(new_n214), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n225), .c(new_n124), .d(new_n216), .o1(new_n233));
  nano22aa1n02x4               g138(.a(new_n227), .b(new_n213), .c(new_n228), .out0(new_n234));
  oai022aa1n02x5               g139(.a(\a[18] ), .b(\b[17] ), .c(\b[18] ), .d(\a[19] ), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(\a[19] ), .d(\b[18] ), .o1(new_n236));
  nand02aa1n02x5               g141(.a(new_n233), .b(new_n236), .o1(new_n237));
  aoi012aa1d24x5               g142(.a(new_n227), .b(new_n213), .c(new_n228), .o1(new_n238));
  xorc02aa1n12x5               g143(.a(\a[19] ), .b(\b[18] ), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n238), .d(new_n233), .o1(\s[19] ));
  xnrc02aa1n02x5               g145(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g146(.a(\b[18] ), .b(\a[19] ), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n238), .o1(new_n244));
  aoai13aa1n02x7               g149(.a(new_n239), .b(new_n244), .c(new_n212), .d(new_n232), .o1(new_n245));
  nor042aa1n06x5               g150(.a(\b[19] ), .b(\a[20] ), .o1(new_n246));
  and002aa1n12x5               g151(.a(\b[19] ), .b(\a[20] ), .o(new_n247));
  nor042aa1n12x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[18] ), .b(\a[19] ), .out0(new_n249));
  norp03aa1n02x5               g154(.a(new_n247), .b(new_n246), .c(new_n242), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n249), .c(new_n233), .d(new_n238), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n248), .c(new_n245), .d(new_n243), .o1(\s[20] ));
  nano32aa1n03x7               g157(.a(new_n249), .b(new_n248), .c(new_n215), .d(new_n229), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n225), .c(new_n124), .d(new_n216), .o1(new_n254));
  norp03aa1n02x5               g159(.a(new_n243), .b(new_n247), .c(new_n246), .o1(new_n255));
  nand02aa1n08x5               g160(.a(\b[20] ), .b(\a[21] ), .o1(new_n256));
  nor002aa1d32x5               g161(.a(\b[20] ), .b(\a[21] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  oai112aa1n02x5               g163(.a(new_n258), .b(new_n256), .c(\b[19] ), .d(\a[20] ), .o1(new_n259));
  aoi113aa1n02x5               g164(.a(new_n255), .b(new_n259), .c(new_n244), .d(new_n239), .e(new_n248), .o1(new_n260));
  nand02aa1n02x5               g165(.a(new_n254), .b(new_n260), .o1(new_n261));
  tech160nm_fixnrc02aa1n04x5   g166(.a(\b[19] ), .b(\a[20] ), .out0(new_n262));
  oab012aa1n09x5               g167(.a(new_n246), .b(new_n243), .c(new_n247), .out0(new_n263));
  oai013aa1d12x5               g168(.a(new_n263), .b(new_n249), .c(new_n262), .d(new_n238), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  nanb02aa1n02x5               g170(.a(new_n257), .b(new_n256), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n02x5               g172(.a(new_n261), .b(new_n267), .c(new_n265), .d(new_n254), .o1(\s[21] ));
  aoai13aa1n02x5               g173(.a(new_n267), .b(new_n264), .c(new_n212), .d(new_n253), .o1(new_n269));
  nor002aa1d32x5               g174(.a(\b[21] ), .b(\a[22] ), .o1(new_n270));
  nand02aa1d28x5               g175(.a(\b[21] ), .b(\a[22] ), .o1(new_n271));
  norb02aa1n02x5               g176(.a(new_n271), .b(new_n270), .out0(new_n272));
  norb03aa1n02x5               g177(.a(new_n271), .b(new_n257), .c(new_n270), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n266), .c(new_n254), .d(new_n265), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .d(new_n258), .o1(\s[22] ));
  nona23aa1d24x5               g180(.a(new_n256), .b(new_n271), .c(new_n270), .d(new_n257), .out0(new_n276));
  nano32aa1n02x4               g181(.a(new_n276), .b(new_n232), .c(new_n239), .d(new_n248), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n225), .c(new_n124), .d(new_n216), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n276), .o1(new_n279));
  aoi012aa1n02x7               g184(.a(new_n270), .b(new_n257), .c(new_n271), .o1(new_n280));
  inv020aa1n04x5               g185(.a(new_n280), .o1(new_n281));
  aoi012aa1n06x5               g186(.a(new_n281), .b(new_n264), .c(new_n279), .o1(new_n282));
  nor002aa1d32x5               g187(.a(\b[22] ), .b(\a[23] ), .o1(new_n283));
  tech160nm_finand02aa1n03p5x5 g188(.a(\b[22] ), .b(\a[23] ), .o1(new_n284));
  norb02aa1n03x5               g189(.a(new_n284), .b(new_n283), .out0(new_n285));
  nanb03aa1n02x5               g190(.a(new_n270), .b(new_n271), .c(new_n257), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n283), .o1(new_n287));
  nano32aa1n02x4               g192(.a(new_n270), .b(new_n284), .c(new_n286), .d(new_n287), .out0(new_n288));
  oai112aa1n03x5               g193(.a(new_n278), .b(new_n288), .c(new_n276), .d(new_n265), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n285), .c(new_n278), .d(new_n282), .o1(\s[23] ));
  inv000aa1n02x5               g195(.a(new_n282), .o1(new_n291));
  aoai13aa1n02x7               g196(.a(new_n285), .b(new_n291), .c(new_n212), .d(new_n277), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[24] ), .b(\b[23] ), .out0(new_n293));
  nanb02aa1n06x5               g198(.a(new_n283), .b(new_n284), .out0(new_n294));
  orn002aa1n24x5               g199(.a(\a[24] ), .b(\b[23] ), .o(new_n295));
  nanp02aa1n04x5               g200(.a(\b[23] ), .b(\a[24] ), .o1(new_n296));
  nano22aa1n02x4               g201(.a(new_n283), .b(new_n295), .c(new_n296), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n278), .d(new_n282), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n292), .d(new_n287), .o1(\s[24] ));
  nano22aa1n12x5               g204(.a(new_n294), .b(new_n295), .c(new_n296), .out0(new_n300));
  nanb02aa1n06x5               g205(.a(new_n276), .b(new_n300), .out0(new_n301));
  nano32aa1n02x5               g206(.a(new_n301), .b(new_n248), .c(new_n232), .d(new_n239), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n225), .c(new_n124), .d(new_n216), .o1(new_n303));
  nano32aa1n02x4               g208(.a(new_n276), .b(new_n296), .c(new_n285), .d(new_n295), .out0(new_n304));
  nand22aa1n03x5               g209(.a(new_n264), .b(new_n304), .o1(new_n305));
  nanb03aa1n02x5               g210(.a(new_n280), .b(new_n293), .c(new_n285), .out0(new_n306));
  nand42aa1n03x5               g211(.a(\b[24] ), .b(\a[25] ), .o1(new_n307));
  nor022aa1n16x5               g212(.a(\b[24] ), .b(\a[25] ), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n308), .o1(new_n309));
  oai112aa1n02x5               g214(.a(new_n309), .b(new_n307), .c(\b[23] ), .d(\a[24] ), .o1(new_n310));
  aoi013aa1n02x4               g215(.a(new_n310), .b(new_n296), .c(new_n295), .d(new_n283), .o1(new_n311));
  nanp03aa1n02x5               g216(.a(new_n305), .b(new_n306), .c(new_n311), .o1(new_n312));
  nanb03aa1n12x5               g217(.a(new_n238), .b(new_n239), .c(new_n248), .out0(new_n313));
  aob012aa1n02x5               g218(.a(new_n295), .b(new_n283), .c(new_n296), .out0(new_n314));
  aoi012aa1n09x5               g219(.a(new_n314), .b(new_n300), .c(new_n281), .o1(new_n315));
  aoai13aa1n12x5               g220(.a(new_n315), .b(new_n301), .c(new_n313), .d(new_n263), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n307), .b(new_n308), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n212), .d(new_n302), .o1(new_n319));
  oaib12aa1n02x5               g224(.a(new_n319), .b(new_n312), .c(new_n303), .out0(\s[25] ));
  aoai13aa1n03x5               g225(.a(new_n317), .b(new_n316), .c(new_n212), .d(new_n302), .o1(new_n321));
  nor002aa1n03x5               g226(.a(\b[25] ), .b(\a[26] ), .o1(new_n322));
  and002aa1n06x5               g227(.a(\b[25] ), .b(\a[26] ), .o(new_n323));
  norp02aa1n02x5               g228(.a(new_n323), .b(new_n322), .o1(new_n324));
  inv000aa1n02x5               g229(.a(new_n316), .o1(new_n325));
  norp03aa1n02x5               g230(.a(new_n323), .b(new_n322), .c(new_n308), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n318), .c(new_n303), .d(new_n325), .o1(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n324), .c(new_n321), .d(new_n309), .o1(\s[26] ));
  nano23aa1n06x5               g233(.a(new_n323), .b(new_n322), .c(new_n309), .d(new_n307), .out0(new_n329));
  inv000aa1n02x5               g234(.a(new_n329), .o1(new_n330));
  nano32aa1n03x7               g235(.a(new_n330), .b(new_n253), .c(new_n279), .d(new_n300), .out0(new_n331));
  aoai13aa1n12x5               g236(.a(new_n331), .b(new_n225), .c(new_n124), .d(new_n216), .o1(new_n332));
  oab012aa1n02x4               g237(.a(new_n322), .b(new_n309), .c(new_n323), .out0(new_n333));
  aobi12aa1n12x5               g238(.a(new_n333), .b(new_n316), .c(new_n329), .out0(new_n334));
  nor042aa1d18x5               g239(.a(\b[26] ), .b(\a[27] ), .o1(new_n335));
  tech160nm_finand02aa1n05x5   g240(.a(\b[26] ), .b(\a[27] ), .o1(new_n336));
  norb02aa1n15x5               g241(.a(new_n336), .b(new_n335), .out0(new_n337));
  norb03aa1n02x5               g242(.a(new_n336), .b(new_n322), .c(new_n335), .out0(new_n338));
  oai013aa1n02x4               g243(.a(new_n338), .b(new_n309), .c(new_n323), .d(new_n322), .o1(new_n339));
  aoi012aa1n03x5               g244(.a(new_n339), .b(new_n316), .c(new_n329), .o1(new_n340));
  nanp02aa1n03x5               g245(.a(new_n332), .b(new_n340), .o1(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n337), .c(new_n332), .d(new_n334), .o1(\s[27] ));
  inv000aa1n12x5               g247(.a(new_n335), .o1(new_n343));
  aoai13aa1n02x7               g248(.a(new_n333), .b(new_n330), .c(new_n305), .d(new_n315), .o1(new_n344));
  aoai13aa1n02x5               g249(.a(new_n337), .b(new_n344), .c(new_n212), .d(new_n331), .o1(new_n345));
  nor042aa1n04x5               g250(.a(\b[27] ), .b(\a[28] ), .o1(new_n346));
  and002aa1n12x5               g251(.a(\b[27] ), .b(\a[28] ), .o(new_n347));
  norp02aa1n03x5               g252(.a(new_n347), .b(new_n346), .o1(new_n348));
  inv000aa1n02x5               g253(.a(new_n337), .o1(new_n349));
  norp03aa1n02x5               g254(.a(new_n347), .b(new_n346), .c(new_n335), .o1(new_n350));
  aoai13aa1n03x5               g255(.a(new_n350), .b(new_n349), .c(new_n332), .d(new_n334), .o1(new_n351));
  aoai13aa1n03x5               g256(.a(new_n351), .b(new_n348), .c(new_n345), .d(new_n343), .o1(\s[28] ));
  nano23aa1n02x4               g257(.a(new_n347), .b(new_n346), .c(new_n343), .d(new_n336), .out0(new_n353));
  aoai13aa1n02x5               g258(.a(new_n353), .b(new_n344), .c(new_n212), .d(new_n331), .o1(new_n354));
  inv000aa1n02x5               g259(.a(new_n353), .o1(new_n355));
  norp03aa1n02x5               g260(.a(new_n343), .b(new_n347), .c(new_n346), .o1(new_n356));
  oai022aa1n02x5               g261(.a(\a[28] ), .b(\b[27] ), .c(\b[28] ), .d(\a[29] ), .o1(new_n357));
  aoi112aa1n02x5               g262(.a(new_n356), .b(new_n357), .c(\a[29] ), .d(\b[28] ), .o1(new_n358));
  aoai13aa1n03x5               g263(.a(new_n358), .b(new_n355), .c(new_n332), .d(new_n334), .o1(new_n359));
  oab012aa1n06x5               g264(.a(new_n346), .b(new_n343), .c(new_n347), .out0(new_n360));
  xorc02aa1n02x5               g265(.a(\a[29] ), .b(\b[28] ), .out0(new_n361));
  aoai13aa1n03x5               g266(.a(new_n359), .b(new_n361), .c(new_n354), .d(new_n360), .o1(\s[29] ));
  xorb03aa1n02x5               g267(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g268(.a(new_n361), .b(new_n348), .c(new_n337), .o(new_n364));
  aoai13aa1n02x5               g269(.a(new_n364), .b(new_n344), .c(new_n212), .d(new_n331), .o1(new_n365));
  tech160nm_fioaoi03aa1n03p5x5 g270(.a(\a[29] ), .b(\b[28] ), .c(new_n360), .o1(new_n366));
  inv000aa1d42x5               g271(.a(new_n366), .o1(new_n367));
  norp02aa1n02x5               g272(.a(\b[29] ), .b(\a[30] ), .o1(new_n368));
  nanp02aa1n02x5               g273(.a(\b[29] ), .b(\a[30] ), .o1(new_n369));
  norb02aa1n02x5               g274(.a(new_n369), .b(new_n368), .out0(new_n370));
  inv000aa1n02x5               g275(.a(new_n364), .o1(new_n371));
  oai012aa1n02x5               g276(.a(new_n361), .b(new_n356), .c(new_n346), .o1(new_n372));
  oai022aa1n02x5               g277(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n373));
  nano22aa1n02x4               g278(.a(new_n373), .b(new_n372), .c(new_n369), .out0(new_n374));
  aoai13aa1n03x5               g279(.a(new_n374), .b(new_n371), .c(new_n332), .d(new_n334), .o1(new_n375));
  aoai13aa1n03x5               g280(.a(new_n375), .b(new_n370), .c(new_n365), .d(new_n367), .o1(\s[30] ));
  nano32aa1n02x4               g281(.a(new_n349), .b(new_n370), .c(new_n348), .d(new_n361), .out0(new_n377));
  aoai13aa1n02x5               g282(.a(new_n377), .b(new_n344), .c(new_n212), .d(new_n331), .o1(new_n378));
  xnrc02aa1n02x5               g283(.a(\b[30] ), .b(\a[31] ), .out0(new_n379));
  inv000aa1d42x5               g284(.a(new_n379), .o1(new_n380));
  inv000aa1n02x5               g285(.a(new_n377), .o1(new_n381));
  aoi112aa1n03x4               g286(.a(new_n368), .b(new_n379), .c(new_n366), .d(new_n370), .o1(new_n382));
  aoai13aa1n03x5               g287(.a(new_n382), .b(new_n381), .c(new_n332), .d(new_n334), .o1(new_n383));
  aoi012aa1n02x5               g288(.a(new_n368), .b(new_n366), .c(new_n369), .o1(new_n384));
  aoai13aa1n03x5               g289(.a(new_n383), .b(new_n380), .c(new_n378), .d(new_n384), .o1(\s[31] ));
  xnbna2aa1n03x5               g290(.a(new_n135), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  tech160nm_fiao0012aa1n02p5x5 g291(.a(new_n135), .b(new_n101), .c(new_n99), .o(new_n387));
  xobna2aa1n03x5               g292(.a(new_n104), .b(new_n387), .c(new_n106), .out0(\s[4] ));
  nanp02aa1n06x5               g293(.a(\b[4] ), .b(\a[5] ), .o1(new_n389));
  nanb02aa1n06x5               g294(.a(new_n115), .b(new_n389), .out0(new_n390));
  inv000aa1d42x5               g295(.a(new_n390), .o1(new_n391));
  aoi112aa1n03x5               g296(.a(new_n390), .b(new_n102), .c(new_n103), .d(new_n105), .o1(new_n392));
  nanp02aa1n03x5               g297(.a(new_n108), .b(new_n392), .o1(new_n393));
  aoai13aa1n02x5               g298(.a(new_n393), .b(new_n391), .c(new_n108), .d(new_n109), .o1(\s[5] ));
  xorc02aa1n02x5               g299(.a(\a[6] ), .b(\b[5] ), .out0(new_n395));
  xobna2aa1n03x5               g300(.a(new_n395), .b(new_n393), .c(new_n389), .out0(\s[6] ));
  norb02aa1n02x5               g301(.a(new_n113), .b(new_n112), .out0(new_n397));
  and002aa1n02x5               g302(.a(\b[5] ), .b(\a[6] ), .o(new_n398));
  inv000aa1d42x5               g303(.a(new_n398), .o1(new_n399));
  aob012aa1n03x5               g304(.a(new_n395), .b(new_n393), .c(new_n389), .out0(new_n400));
  xobna2aa1n03x5               g305(.a(new_n397), .b(new_n400), .c(new_n399), .out0(\s[7] ));
  norb02aa1n02x5               g306(.a(new_n111), .b(new_n110), .out0(new_n402));
  aoi013aa1n02x4               g307(.a(new_n112), .b(new_n400), .c(new_n399), .d(new_n113), .o1(new_n403));
  nona22aa1n02x4               g308(.a(new_n111), .b(new_n112), .c(new_n110), .out0(new_n404));
  aoi013aa1n02x4               g309(.a(new_n404), .b(new_n400), .c(new_n399), .d(new_n397), .o1(new_n405));
  oabi12aa1n03x5               g310(.a(new_n405), .b(new_n403), .c(new_n402), .out0(\s[8] ));
  aoi112aa1n02x5               g311(.a(new_n125), .b(new_n110), .c(new_n111), .d(new_n112), .o1(new_n407));
  aobi12aa1n02x5               g312(.a(new_n407), .b(new_n121), .c(new_n114), .out0(new_n408));
  ao0022aa1n03x5               g313(.a(new_n124), .b(new_n125), .c(new_n408), .d(new_n140), .o(\s[9] ));
endmodule

