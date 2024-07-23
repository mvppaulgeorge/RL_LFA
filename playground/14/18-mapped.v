// Benchmark "adder" written by ABC on Wed Jul 17 19:16:30 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n159, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n175, new_n176,
    new_n177, new_n178, new_n180, new_n181, new_n182, new_n183, new_n184,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n191, new_n192,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n199, new_n200,
    new_n201, new_n202, new_n203, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n242, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n284, new_n285,
    new_n286, new_n287, new_n288, new_n289, new_n290, new_n291, new_n292,
    new_n293, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n324, new_n325, new_n326, new_n327, new_n328, new_n329, new_n331,
    new_n332, new_n333, new_n334, new_n335, new_n336, new_n337, new_n338,
    new_n339, new_n340, new_n341, new_n343, new_n344, new_n345, new_n346,
    new_n347, new_n348, new_n349, new_n350, new_n351, new_n353, new_n354,
    new_n355, new_n356, new_n357, new_n358, new_n359, new_n360, new_n361,
    new_n362, new_n364, new_n366, new_n367, new_n368, new_n369, new_n370,
    new_n371, new_n372, new_n373, new_n374, new_n375, new_n377, new_n378,
    new_n379, new_n380, new_n381, new_n382, new_n383, new_n384, new_n385,
    new_n386, new_n387, new_n390, new_n392, new_n393, new_n394, new_n396,
    new_n397, new_n399, new_n401, new_n402, new_n404, new_n405;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  oai112aa1n06x5               g003(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n99));
  nand42aa1n06x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n09x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand02aa1d08x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanb03aa1d18x5               g007(.a(new_n101), .b(new_n102), .c(new_n100), .out0(new_n103));
  orn002aa1n24x5               g008(.a(\a[4] ), .b(\b[3] ), .o(new_n104));
  nand42aa1d28x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nano22aa1n03x7               g010(.a(new_n101), .b(new_n104), .c(new_n105), .out0(new_n106));
  oaib12aa1n18x5               g011(.a(new_n106), .b(new_n103), .c(new_n99), .out0(new_n107));
  nand42aa1n08x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nor022aa1n12x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  inv040aa1n04x5               g014(.a(new_n109), .o1(new_n110));
  oai112aa1n03x5               g015(.a(new_n110), .b(new_n108), .c(\b[5] ), .d(\a[6] ), .o1(new_n111));
  tech160nm_fioai012aa1n04x5   g016(.a(new_n105), .b(\b[6] ), .c(\a[7] ), .o1(new_n112));
  norp02aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand42aa1n16x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  aoi022aa1d24x5               g019(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n115));
  nanb03aa1n03x5               g020(.a(new_n113), .b(new_n115), .c(new_n114), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n116), .b(new_n112), .c(new_n111), .o1(new_n117));
  oaih22aa1d12x5               g022(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n118));
  nor042aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n114), .b(new_n113), .c(new_n119), .o1(new_n120));
  nor002aa1d32x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nand02aa1d12x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  norb03aa1n03x5               g027(.a(new_n122), .b(new_n121), .c(new_n109), .out0(new_n123));
  nanp02aa1n06x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nanp03aa1n02x5               g029(.a(new_n122), .b(new_n114), .c(new_n124), .o1(new_n125));
  oai013aa1n03x5               g030(.a(new_n120), .b(new_n123), .c(new_n125), .d(new_n118), .o1(new_n126));
  xorc02aa1n06x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n117), .d(new_n107), .o1(new_n128));
  xorc02aa1n12x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  norb02aa1n06x5               g035(.a(new_n102), .b(new_n101), .out0(new_n131));
  oai112aa1n02x7               g036(.a(new_n104), .b(new_n105), .c(\b[2] ), .d(\a[3] ), .o1(new_n132));
  aoi013aa1n06x5               g037(.a(new_n132), .b(new_n131), .c(new_n100), .d(new_n99), .o1(new_n133));
  norb03aa1n03x5               g038(.a(new_n108), .b(new_n121), .c(new_n109), .out0(new_n134));
  norb02aa1n09x5               g039(.a(new_n114), .b(new_n113), .out0(new_n135));
  nand22aa1n03x5               g040(.a(new_n124), .b(new_n122), .o1(new_n136));
  nona23aa1n09x5               g041(.a(new_n134), .b(new_n135), .c(new_n136), .d(new_n112), .out0(new_n137));
  nona22aa1n02x4               g042(.a(new_n122), .b(new_n109), .c(new_n121), .out0(new_n138));
  nano32aa1n09x5               g043(.a(new_n118), .b(new_n124), .c(new_n122), .d(new_n114), .out0(new_n139));
  aoi022aa1n06x5               g044(.a(new_n139), .b(new_n138), .c(new_n114), .d(new_n118), .o1(new_n140));
  oai012aa1d24x5               g045(.a(new_n140), .b(new_n133), .c(new_n137), .o1(new_n141));
  aoai13aa1n06x5               g046(.a(new_n129), .b(new_n97), .c(new_n141), .d(new_n127), .o1(new_n142));
  tech160nm_fioaoi03aa1n03p5x5 g047(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  nor022aa1n16x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  nand42aa1n08x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n142), .c(new_n144), .out0(\s[11] ));
  nand22aa1n03x5               g053(.a(new_n128), .b(new_n98), .o1(new_n149));
  aoai13aa1n03x5               g054(.a(new_n147), .b(new_n143), .c(new_n149), .d(new_n129), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n145), .o1(new_n151));
  inv000aa1n02x5               g056(.a(new_n147), .o1(new_n152));
  aoai13aa1n03x5               g057(.a(new_n151), .b(new_n152), .c(new_n142), .d(new_n144), .o1(new_n153));
  nor042aa1n04x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nand42aa1n10x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  inv000aa1d42x5               g061(.a(\a[11] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[10] ), .o1(new_n158));
  aboi22aa1n03x5               g063(.a(new_n154), .b(new_n155), .c(new_n157), .d(new_n158), .out0(new_n159));
  aoi022aa1n02x7               g064(.a(new_n153), .b(new_n156), .c(new_n150), .d(new_n159), .o1(\s[12] ));
  nona23aa1n03x5               g065(.a(new_n155), .b(new_n146), .c(new_n145), .d(new_n154), .out0(new_n161));
  nano22aa1n03x7               g066(.a(new_n161), .b(new_n127), .c(new_n129), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n126), .c(new_n117), .d(new_n107), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n155), .b(new_n154), .c(new_n157), .d(new_n158), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[9] ), .o1(new_n165));
  nanb02aa1n03x5               g070(.a(\a[10] ), .b(new_n165), .out0(new_n166));
  nand22aa1n04x5               g071(.a(\b[9] ), .b(\a[10] ), .o1(new_n167));
  oai112aa1n04x5               g072(.a(new_n166), .b(new_n167), .c(\b[8] ), .d(\a[9] ), .o1(new_n168));
  tech160nm_fiaoi012aa1n05x5   g073(.a(new_n145), .b(\a[10] ), .c(\b[9] ), .o1(new_n169));
  nano22aa1n03x7               g074(.a(new_n154), .b(new_n146), .c(new_n155), .out0(new_n170));
  nand03aa1n06x5               g075(.a(new_n170), .b(new_n168), .c(new_n169), .o1(new_n171));
  nanp02aa1n06x5               g076(.a(new_n171), .b(new_n164), .o1(new_n172));
  nanb02aa1n03x5               g077(.a(new_n172), .b(new_n163), .out0(new_n173));
  nor002aa1d32x5               g078(.a(\b[12] ), .b(\a[13] ), .o1(new_n174));
  nand42aa1n16x5               g079(.a(\b[12] ), .b(\a[13] ), .o1(new_n175));
  norb02aa1n06x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  inv030aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  and003aa1n02x5               g082(.a(new_n171), .b(new_n164), .c(new_n177), .o(new_n178));
  aoi022aa1n02x5               g083(.a(new_n173), .b(new_n176), .c(new_n163), .d(new_n178), .o1(\s[13] ));
  inv000aa1n06x5               g084(.a(new_n174), .o1(new_n180));
  aoai13aa1n03x5               g085(.a(new_n176), .b(new_n172), .c(new_n141), .d(new_n162), .o1(new_n181));
  nor022aa1n12x5               g086(.a(\b[13] ), .b(\a[14] ), .o1(new_n182));
  nand42aa1d28x5               g087(.a(\b[13] ), .b(\a[14] ), .o1(new_n183));
  norb02aa1n09x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n181), .c(new_n180), .out0(\s[14] ));
  oaoi03aa1n02x5               g090(.a(\a[14] ), .b(\b[13] ), .c(new_n180), .o1(new_n186));
  inv000aa1n02x5               g091(.a(new_n186), .o1(new_n187));
  nano23aa1d15x5               g092(.a(new_n174), .b(new_n182), .c(new_n183), .d(new_n175), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n172), .c(new_n141), .d(new_n162), .o1(new_n189));
  nor042aa1d18x5               g094(.a(\b[14] ), .b(\a[15] ), .o1(new_n190));
  nand02aa1d20x5               g095(.a(\b[14] ), .b(\a[15] ), .o1(new_n191));
  norb02aa1d27x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n189), .c(new_n187), .out0(\s[15] ));
  aoai13aa1n06x5               g098(.a(new_n192), .b(new_n186), .c(new_n173), .d(new_n188), .o1(new_n194));
  inv000aa1d42x5               g099(.a(new_n190), .o1(new_n195));
  inv000aa1d42x5               g100(.a(new_n192), .o1(new_n196));
  aoai13aa1n03x5               g101(.a(new_n195), .b(new_n196), .c(new_n189), .d(new_n187), .o1(new_n197));
  xorc02aa1n12x5               g102(.a(\a[16] ), .b(\b[15] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(\a[16] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[15] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  nanp02aa1n24x5               g106(.a(\b[15] ), .b(\a[16] ), .o1(new_n202));
  aoi012aa1n02x5               g107(.a(new_n190), .b(new_n201), .c(new_n202), .o1(new_n203));
  aoi022aa1n02x5               g108(.a(new_n197), .b(new_n198), .c(new_n194), .d(new_n203), .o1(\s[16] ));
  nano23aa1n03x7               g109(.a(new_n145), .b(new_n154), .c(new_n155), .d(new_n146), .out0(new_n205));
  nand23aa1n09x5               g110(.a(new_n188), .b(new_n192), .c(new_n198), .o1(new_n206));
  nano32aa1d12x5               g111(.a(new_n206), .b(new_n205), .c(new_n129), .d(new_n127), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n141), .b(new_n207), .o1(new_n208));
  aoi012aa1n12x5               g113(.a(new_n126), .b(new_n117), .c(new_n107), .o1(new_n209));
  nano32aa1d12x5               g114(.a(new_n177), .b(new_n198), .c(new_n184), .d(new_n192), .out0(new_n210));
  nand22aa1n04x5               g115(.a(new_n210), .b(new_n162), .o1(new_n211));
  nona22aa1n09x5               g116(.a(new_n183), .b(new_n182), .c(new_n174), .out0(new_n212));
  inv040aa1n02x5               g117(.a(new_n202), .o1(new_n213));
  oai012aa1n06x5               g118(.a(new_n183), .b(\b[14] ), .c(\a[15] ), .o1(new_n214));
  tech160nm_fioai012aa1n04x5   g119(.a(new_n191), .b(\b[15] ), .c(\a[16] ), .o1(new_n215));
  norp03aa1n06x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .o1(new_n216));
  oai022aa1n06x5               g121(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n217));
  aoi022aa1n12x5               g122(.a(new_n216), .b(new_n212), .c(new_n202), .d(new_n217), .o1(new_n218));
  inv030aa1n02x5               g123(.a(new_n218), .o1(new_n219));
  aoi012aa1n12x5               g124(.a(new_n219), .b(new_n172), .c(new_n210), .o1(new_n220));
  oai012aa1n12x5               g125(.a(new_n220), .b(new_n209), .c(new_n211), .o1(new_n221));
  tech160nm_fixorc02aa1n02p5x5 g126(.a(\a[17] ), .b(\b[16] ), .out0(new_n222));
  tech160nm_fiao0012aa1n02p5x5 g127(.a(new_n222), .b(new_n202), .c(new_n217), .o(new_n223));
  aoi122aa1n02x5               g128(.a(new_n223), .b(new_n212), .c(new_n216), .d(new_n172), .e(new_n210), .o1(new_n224));
  aoi022aa1n02x5               g129(.a(new_n221), .b(new_n222), .c(new_n224), .d(new_n208), .o1(\s[17] ));
  inv000aa1d42x5               g130(.a(\a[17] ), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(\b[16] ), .b(new_n226), .out0(new_n227));
  aoai13aa1n12x5               g132(.a(new_n218), .b(new_n206), .c(new_n171), .d(new_n164), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n222), .b(new_n228), .c(new_n141), .d(new_n207), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[18] ), .b(\b[17] ), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n229), .c(new_n227), .out0(\s[18] ));
  inv040aa1d32x5               g136(.a(\a[18] ), .o1(new_n232));
  xroi22aa1d06x4               g137(.a(new_n226), .b(\b[16] ), .c(new_n232), .d(\b[17] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n228), .c(new_n141), .d(new_n207), .o1(new_n234));
  oaoi03aa1n12x5               g139(.a(\a[18] ), .b(\b[17] ), .c(new_n227), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  nor002aa1d32x5               g141(.a(\b[18] ), .b(\a[19] ), .o1(new_n237));
  nand42aa1n08x5               g142(.a(\b[18] ), .b(\a[19] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  xnbna2aa1n03x5               g144(.a(new_n239), .b(new_n234), .c(new_n236), .out0(\s[19] ));
  xnrc02aa1n02x5               g145(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g146(.a(new_n239), .b(new_n235), .c(new_n221), .d(new_n233), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n237), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n239), .o1(new_n244));
  aoai13aa1n03x5               g149(.a(new_n243), .b(new_n244), .c(new_n234), .d(new_n236), .o1(new_n245));
  nor042aa1n04x5               g150(.a(\b[19] ), .b(\a[20] ), .o1(new_n246));
  nand42aa1n16x5               g151(.a(\b[19] ), .b(\a[20] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  inv000aa1d42x5               g153(.a(\a[19] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(\b[18] ), .o1(new_n250));
  aboi22aa1n03x5               g155(.a(new_n246), .b(new_n247), .c(new_n249), .d(new_n250), .out0(new_n251));
  aoi022aa1n03x5               g156(.a(new_n245), .b(new_n248), .c(new_n242), .d(new_n251), .o1(\s[20] ));
  nano23aa1n03x7               g157(.a(new_n237), .b(new_n246), .c(new_n247), .d(new_n238), .out0(new_n253));
  and003aa1n02x5               g158(.a(new_n253), .b(new_n230), .c(new_n222), .o(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n228), .c(new_n141), .d(new_n207), .o1(new_n255));
  nanp02aa1n04x5               g160(.a(\b[17] ), .b(\a[18] ), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  oaih22aa1n04x5               g162(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n258));
  tech160nm_fiaoi012aa1n04x5   g163(.a(new_n237), .b(\a[18] ), .c(\b[17] ), .o1(new_n259));
  nano22aa1n03x7               g164(.a(new_n246), .b(new_n238), .c(new_n247), .out0(new_n260));
  oai112aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n258), .d(new_n257), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n247), .b(new_n246), .c(new_n249), .d(new_n250), .o1(new_n262));
  nand02aa1d10x5               g167(.a(new_n261), .b(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n255), .b(new_n264), .o1(new_n265));
  nor002aa1d32x5               g170(.a(\b[20] ), .b(\a[21] ), .o1(new_n266));
  nand42aa1n16x5               g171(.a(\b[20] ), .b(\a[21] ), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n267), .b(new_n266), .out0(new_n268));
  nanb03aa1n02x5               g173(.a(new_n246), .b(new_n247), .c(new_n238), .out0(new_n269));
  nano32aa1n06x5               g174(.a(new_n269), .b(new_n258), .c(new_n243), .d(new_n256), .out0(new_n270));
  inv000aa1n02x5               g175(.a(new_n268), .o1(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n262), .c(new_n271), .out0(new_n272));
  aoi022aa1n02x5               g177(.a(new_n265), .b(new_n268), .c(new_n255), .d(new_n272), .o1(\s[21] ));
  aoai13aa1n03x5               g178(.a(new_n268), .b(new_n263), .c(new_n221), .d(new_n254), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n266), .o1(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n271), .c(new_n255), .d(new_n264), .o1(new_n276));
  nor002aa1d32x5               g181(.a(\b[21] ), .b(\a[22] ), .o1(new_n277));
  nand42aa1d28x5               g182(.a(\b[21] ), .b(\a[22] ), .o1(new_n278));
  norb02aa1n02x5               g183(.a(new_n278), .b(new_n277), .out0(new_n279));
  inv000aa1d42x5               g184(.a(\a[21] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(\b[20] ), .o1(new_n281));
  aboi22aa1n03x5               g186(.a(new_n277), .b(new_n278), .c(new_n280), .d(new_n281), .out0(new_n282));
  aoi022aa1n02x7               g187(.a(new_n276), .b(new_n279), .c(new_n274), .d(new_n282), .o1(\s[22] ));
  nona23aa1n02x4               g188(.a(new_n278), .b(new_n267), .c(new_n266), .d(new_n277), .out0(new_n284));
  nano32aa1n02x4               g189(.a(new_n284), .b(new_n253), .c(new_n230), .d(new_n222), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n228), .c(new_n141), .d(new_n207), .o1(new_n286));
  nano23aa1d15x5               g191(.a(new_n266), .b(new_n277), .c(new_n278), .d(new_n267), .out0(new_n287));
  inv040aa1n08x5               g192(.a(new_n277), .o1(new_n288));
  oai112aa1n06x5               g193(.a(new_n288), .b(new_n278), .c(\b[20] ), .d(\a[21] ), .o1(new_n289));
  aoi022aa1d24x5               g194(.a(new_n263), .b(new_n287), .c(new_n278), .d(new_n289), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n286), .b(new_n290), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[23] ), .b(\b[22] ), .out0(new_n292));
  aoi122aa1n02x5               g197(.a(new_n292), .b(new_n278), .c(new_n289), .d(new_n263), .e(new_n287), .o1(new_n293));
  aoi022aa1n02x5               g198(.a(new_n291), .b(new_n292), .c(new_n286), .d(new_n293), .o1(\s[23] ));
  inv000aa1d42x5               g199(.a(new_n290), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n292), .b(new_n295), .c(new_n221), .d(new_n285), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\a[23] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[22] ), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n298), .b(new_n297), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n292), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n299), .b(new_n300), .c(new_n286), .d(new_n290), .o1(new_n301));
  tech160nm_fixorc02aa1n02p5x5 g206(.a(\a[24] ), .b(\b[23] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n297), .c(new_n298), .o1(new_n303));
  aoi022aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n296), .d(new_n303), .o1(\s[24] ));
  nand23aa1n03x5               g209(.a(new_n287), .b(new_n292), .c(new_n302), .o1(new_n305));
  nano32aa1n02x4               g210(.a(new_n305), .b(new_n253), .c(new_n230), .d(new_n222), .out0(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n228), .c(new_n141), .d(new_n207), .o1(new_n307));
  nano22aa1n03x7               g212(.a(new_n284), .b(new_n292), .c(new_n302), .out0(new_n308));
  oaib12aa1n03x5               g213(.a(new_n308), .b(new_n270), .c(new_n262), .out0(new_n309));
  and002aa1n02x5               g214(.a(\b[23] ), .b(\a[24] ), .o(new_n310));
  oai012aa1n02x5               g215(.a(new_n278), .b(\b[22] ), .c(\a[23] ), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(\b[22] ), .b(\a[23] ), .o1(new_n312));
  oai012aa1n02x5               g217(.a(new_n312), .b(\b[23] ), .c(\a[24] ), .o1(new_n313));
  nor043aa1n02x5               g218(.a(new_n313), .b(new_n311), .c(new_n310), .o1(new_n314));
  oaoi03aa1n02x5               g219(.a(\a[24] ), .b(\b[23] ), .c(new_n299), .o1(new_n315));
  aoi012aa1n06x5               g220(.a(new_n315), .b(new_n314), .c(new_n289), .o1(new_n316));
  aoai13aa1n06x5               g221(.a(new_n316), .b(new_n305), .c(new_n261), .d(new_n262), .o1(new_n317));
  inv000aa1n02x5               g222(.a(new_n317), .o1(new_n318));
  xorc02aa1n12x5               g223(.a(\a[25] ), .b(\b[24] ), .out0(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  aoi012aa1n02x5               g225(.a(new_n320), .b(new_n307), .c(new_n318), .o1(new_n321));
  aoi112aa1n02x5               g226(.a(new_n315), .b(new_n319), .c(new_n314), .d(new_n289), .o1(new_n322));
  aoi013aa1n02x4               g227(.a(new_n321), .b(new_n309), .c(new_n307), .d(new_n322), .o1(\s[25] ));
  aoai13aa1n03x5               g228(.a(new_n319), .b(new_n317), .c(new_n221), .d(new_n306), .o1(new_n324));
  nor002aa1d32x5               g229(.a(\b[24] ), .b(\a[25] ), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n325), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n320), .c(new_n307), .d(new_n318), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[26] ), .b(\b[25] ), .out0(new_n328));
  norp02aa1n02x5               g233(.a(new_n328), .b(new_n325), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n329), .o1(\s[26] ));
  nanp02aa1n02x5               g235(.a(\b[24] ), .b(\a[25] ), .o1(new_n331));
  tech160nm_fixnrc02aa1n05x5   g236(.a(\b[25] ), .b(\a[26] ), .out0(new_n332));
  nano22aa1n06x5               g237(.a(new_n332), .b(new_n326), .c(new_n331), .out0(new_n333));
  nano32aa1n03x7               g238(.a(new_n305), .b(new_n233), .c(new_n333), .d(new_n253), .out0(new_n334));
  aoai13aa1n12x5               g239(.a(new_n334), .b(new_n228), .c(new_n141), .d(new_n207), .o1(new_n335));
  oao003aa1n02x5               g240(.a(\a[26] ), .b(\b[25] ), .c(new_n326), .carry(new_n336));
  inv000aa1d42x5               g241(.a(new_n336), .o1(new_n337));
  aoi012aa1n12x5               g242(.a(new_n337), .b(new_n317), .c(new_n333), .o1(new_n338));
  nand02aa1n02x5               g243(.a(new_n335), .b(new_n338), .o1(new_n339));
  xorc02aa1n12x5               g244(.a(\a[27] ), .b(\b[26] ), .out0(new_n340));
  aoi112aa1n02x7               g245(.a(new_n340), .b(new_n337), .c(new_n317), .d(new_n333), .o1(new_n341));
  aoi022aa1n03x5               g246(.a(new_n339), .b(new_n340), .c(new_n335), .d(new_n341), .o1(\s[27] ));
  inv000aa1n02x5               g247(.a(new_n333), .o1(new_n343));
  aoai13aa1n04x5               g248(.a(new_n336), .b(new_n343), .c(new_n309), .d(new_n316), .o1(new_n344));
  aoai13aa1n03x5               g249(.a(new_n340), .b(new_n344), .c(new_n221), .d(new_n334), .o1(new_n345));
  norp02aa1n02x5               g250(.a(\b[26] ), .b(\a[27] ), .o1(new_n346));
  inv000aa1d42x5               g251(.a(new_n346), .o1(new_n347));
  inv000aa1d42x5               g252(.a(new_n340), .o1(new_n348));
  aoai13aa1n03x5               g253(.a(new_n347), .b(new_n348), .c(new_n335), .d(new_n338), .o1(new_n349));
  xorc02aa1n02x5               g254(.a(\a[28] ), .b(\b[27] ), .out0(new_n350));
  norp02aa1n02x5               g255(.a(new_n350), .b(new_n346), .o1(new_n351));
  aoi022aa1n03x5               g256(.a(new_n349), .b(new_n350), .c(new_n345), .d(new_n351), .o1(\s[28] ));
  and002aa1n02x5               g257(.a(new_n350), .b(new_n340), .o(new_n353));
  aoai13aa1n03x5               g258(.a(new_n353), .b(new_n344), .c(new_n221), .d(new_n334), .o1(new_n354));
  inv000aa1d42x5               g259(.a(new_n353), .o1(new_n355));
  inv000aa1d42x5               g260(.a(\a[28] ), .o1(new_n356));
  inv000aa1d42x5               g261(.a(\b[27] ), .o1(new_n357));
  aoi112aa1n02x5               g262(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n358));
  aoi012aa1n02x5               g263(.a(new_n358), .b(new_n356), .c(new_n357), .o1(new_n359));
  aoai13aa1n03x5               g264(.a(new_n359), .b(new_n355), .c(new_n335), .d(new_n338), .o1(new_n360));
  xorc02aa1n02x5               g265(.a(\a[29] ), .b(\b[28] ), .out0(new_n361));
  aoi112aa1n02x5               g266(.a(new_n361), .b(new_n358), .c(new_n356), .d(new_n357), .o1(new_n362));
  aoi022aa1n03x5               g267(.a(new_n360), .b(new_n361), .c(new_n354), .d(new_n362), .o1(\s[29] ));
  nanp02aa1n02x5               g268(.a(\b[0] ), .b(\a[1] ), .o1(new_n364));
  xorb03aa1n02x5               g269(.a(new_n364), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g270(.a(new_n348), .b(new_n350), .c(new_n361), .out0(new_n366));
  aoai13aa1n03x5               g271(.a(new_n366), .b(new_n344), .c(new_n221), .d(new_n334), .o1(new_n367));
  inv000aa1d42x5               g272(.a(new_n366), .o1(new_n368));
  inv000aa1d42x5               g273(.a(\a[29] ), .o1(new_n369));
  inv000aa1d42x5               g274(.a(\b[28] ), .o1(new_n370));
  aoi122aa1n02x5               g275(.a(new_n358), .b(new_n370), .c(new_n369), .d(new_n357), .e(new_n356), .o1(new_n371));
  oabi12aa1n02x5               g276(.a(new_n371), .b(new_n369), .c(new_n370), .out0(new_n372));
  aoai13aa1n03x5               g277(.a(new_n372), .b(new_n368), .c(new_n335), .d(new_n338), .o1(new_n373));
  xorc02aa1n02x5               g278(.a(\a[30] ), .b(\b[29] ), .out0(new_n374));
  norb02aa1n02x5               g279(.a(new_n372), .b(new_n374), .out0(new_n375));
  aoi022aa1n03x5               g280(.a(new_n373), .b(new_n374), .c(new_n367), .d(new_n375), .o1(\s[30] ));
  nanb02aa1n02x5               g281(.a(\a[31] ), .b(\b[30] ), .out0(new_n377));
  nanb02aa1n02x5               g282(.a(\b[30] ), .b(\a[31] ), .out0(new_n378));
  nanp02aa1n02x5               g283(.a(new_n378), .b(new_n377), .o1(new_n379));
  nano32aa1n03x7               g284(.a(new_n348), .b(new_n374), .c(new_n350), .d(new_n361), .out0(new_n380));
  aoai13aa1n03x5               g285(.a(new_n380), .b(new_n344), .c(new_n221), .d(new_n334), .o1(new_n381));
  inv000aa1d42x5               g286(.a(new_n380), .o1(new_n382));
  norp02aa1n02x5               g287(.a(\b[29] ), .b(\a[30] ), .o1(new_n383));
  aoi122aa1n02x5               g288(.a(new_n371), .b(\b[29] ), .c(\a[30] ), .d(\b[28] ), .e(\a[29] ), .o1(new_n384));
  norp02aa1n02x5               g289(.a(new_n384), .b(new_n383), .o1(new_n385));
  aoai13aa1n03x5               g290(.a(new_n385), .b(new_n382), .c(new_n335), .d(new_n338), .o1(new_n386));
  nano23aa1n02x4               g291(.a(new_n384), .b(new_n383), .c(new_n377), .d(new_n378), .out0(new_n387));
  aoi022aa1n03x5               g292(.a(new_n386), .b(new_n379), .c(new_n381), .d(new_n387), .o1(\s[31] ));
  xobna2aa1n03x5               g293(.a(new_n131), .b(new_n100), .c(new_n99), .out0(\s[3] ));
  aoi013aa1n02x4               g294(.a(new_n101), .b(new_n99), .c(new_n100), .d(new_n102), .o1(new_n390));
  aoai13aa1n02x5               g295(.a(new_n107), .b(new_n390), .c(new_n105), .d(new_n104), .o1(\s[4] ));
  aoi022aa1n02x5               g296(.a(new_n107), .b(new_n105), .c(new_n110), .d(new_n108), .o1(new_n392));
  nano22aa1n02x4               g297(.a(new_n109), .b(new_n105), .c(new_n108), .out0(new_n393));
  nanp02aa1n02x5               g298(.a(new_n107), .b(new_n393), .o1(new_n394));
  norb02aa1n02x5               g299(.a(new_n394), .b(new_n392), .out0(\s[5] ));
  norb02aa1n02x5               g300(.a(new_n122), .b(new_n121), .out0(new_n396));
  oaib12aa1n02x5               g301(.a(new_n123), .b(new_n133), .c(new_n393), .out0(new_n397));
  aoai13aa1n02x5               g302(.a(new_n397), .b(new_n396), .c(new_n394), .d(new_n110), .o1(\s[6] ));
  norb02aa1n02x5               g303(.a(new_n124), .b(new_n119), .out0(new_n399));
  xobna2aa1n03x5               g304(.a(new_n399), .b(new_n397), .c(new_n122), .out0(\s[7] ));
  aoi013aa1n02x4               g305(.a(new_n119), .b(new_n397), .c(new_n124), .d(new_n122), .o1(new_n401));
  aoi113aa1n02x5               g306(.a(new_n119), .b(new_n135), .c(new_n397), .d(new_n399), .e(new_n122), .o1(new_n402));
  aoib12aa1n02x5               g307(.a(new_n402), .b(new_n135), .c(new_n401), .out0(\s[8] ));
  nanp02aa1n02x5               g308(.a(new_n117), .b(new_n107), .o1(new_n404));
  aoi122aa1n02x5               g309(.a(new_n127), .b(new_n114), .c(new_n118), .d(new_n139), .e(new_n138), .o1(new_n405));
  aoi022aa1n02x5               g310(.a(new_n141), .b(new_n127), .c(new_n405), .d(new_n404), .o1(\s[9] ));
endmodule


