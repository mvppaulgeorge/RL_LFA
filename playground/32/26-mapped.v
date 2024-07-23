// Benchmark "adder" written by ABC on Thu Jul 18 04:34:53 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n355, new_n356, new_n359,
    new_n360, new_n362, new_n364, new_n365, new_n367, new_n368, new_n369,
    new_n371, new_n372, new_n373, new_n375, new_n376;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n02x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nand42aa1n06x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  norb02aa1n06x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  nor042aa1n06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi022aa1n06x5               g006(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n102));
  oai022aa1d18x5               g007(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  oaoi13aa1n12x5               g008(.a(new_n103), .b(new_n100), .c(new_n102), .d(new_n101), .o1(new_n104));
  nand42aa1n16x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  tech160nm_fioai012aa1n04x5   g010(.a(new_n105), .b(\b[5] ), .c(\a[6] ), .o1(new_n106));
  nand42aa1n06x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor002aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand42aa1n10x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nano22aa1n03x7               g014(.a(new_n108), .b(new_n107), .c(new_n109), .out0(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norb02aa1n03x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nona23aa1n12x5               g020(.a(new_n110), .b(new_n115), .c(new_n112), .d(new_n106), .out0(new_n116));
  inv000aa1d42x5               g021(.a(\b[7] ), .o1(new_n117));
  nanb02aa1n06x5               g022(.a(\a[8] ), .b(new_n117), .out0(new_n118));
  nanb03aa1n03x5               g023(.a(new_n113), .b(new_n114), .c(new_n109), .out0(new_n119));
  oai022aa1n02x7               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  nano32aa1n03x7               g025(.a(new_n119), .b(new_n120), .c(new_n118), .d(new_n107), .out0(new_n121));
  aob012aa1n02x5               g026(.a(new_n118), .b(new_n113), .c(new_n109), .out0(new_n122));
  nor042aa1n04x5               g027(.a(new_n121), .b(new_n122), .o1(new_n123));
  oai012aa1n18x5               g028(.a(new_n123), .b(new_n116), .c(new_n104), .o1(new_n124));
  nand42aa1n08x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d24x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  oai012aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n97), .o1(new_n130));
  nano23aa1n03x7               g035(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n131));
  nand22aa1n03x5               g036(.a(new_n124), .b(new_n131), .o1(new_n132));
  nor042aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand02aa1d06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n06x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n132), .c(new_n130), .out0(\s[11] ));
  inv020aa1n10x5               g041(.a(\b[10] ), .o1(new_n137));
  nanb02aa1n12x5               g042(.a(\a[11] ), .b(new_n137), .out0(new_n138));
  aob012aa1n03x5               g043(.a(new_n135), .b(new_n132), .c(new_n130), .out0(new_n139));
  nor002aa1d24x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1d28x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n06x4               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n138), .out0(\s[12] ));
  nona23aa1n03x5               g048(.a(new_n129), .b(new_n125), .c(new_n97), .d(new_n128), .out0(new_n144));
  nano22aa1n03x7               g049(.a(new_n144), .b(new_n135), .c(new_n142), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n124), .b(new_n145), .o1(new_n146));
  inv000aa1d42x5               g051(.a(\b[2] ), .o1(new_n147));
  nanb02aa1n03x5               g052(.a(\a[3] ), .b(new_n147), .out0(new_n148));
  nand42aa1n02x5               g053(.a(new_n148), .b(new_n99), .o1(new_n149));
  inv020aa1n02x5               g054(.a(new_n101), .o1(new_n150));
  tech160nm_finand02aa1n05x5   g055(.a(\b[0] ), .b(\a[1] ), .o1(new_n151));
  aob012aa1n02x5               g056(.a(new_n151), .b(\b[1] ), .c(\a[2] ), .out0(new_n152));
  inv000aa1n02x5               g057(.a(new_n103), .o1(new_n153));
  aoai13aa1n03x5               g058(.a(new_n153), .b(new_n149), .c(new_n152), .d(new_n150), .o1(new_n154));
  nano23aa1n02x4               g059(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n155));
  norp02aa1n02x5               g060(.a(\b[4] ), .b(\a[5] ), .o1(new_n156));
  nano23aa1n02x4               g061(.a(new_n156), .b(new_n113), .c(new_n114), .d(new_n111), .out0(new_n157));
  nand23aa1n03x5               g062(.a(new_n154), .b(new_n155), .c(new_n157), .o1(new_n158));
  nand03aa1n02x5               g063(.a(new_n131), .b(new_n135), .c(new_n142), .o1(new_n159));
  nanb03aa1d24x5               g064(.a(new_n140), .b(new_n141), .c(new_n134), .out0(new_n160));
  oai112aa1n06x5               g065(.a(new_n138), .b(new_n129), .c(new_n128), .d(new_n97), .o1(new_n161));
  aoi012aa1d24x5               g066(.a(new_n140), .b(new_n133), .c(new_n141), .o1(new_n162));
  oai012aa1d24x5               g067(.a(new_n162), .b(new_n161), .c(new_n160), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n159), .c(new_n158), .d(new_n123), .o1(new_n165));
  nor002aa1n20x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nanp02aa1n12x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  nano22aa1n03x7               g073(.a(new_n140), .b(new_n134), .c(new_n141), .out0(new_n169));
  aoi012aa1n02x7               g074(.a(new_n133), .b(\a[10] ), .c(\b[9] ), .o1(new_n170));
  oai112aa1n02x5               g075(.a(new_n169), .b(new_n170), .c(new_n128), .d(new_n97), .o1(new_n171));
  nano22aa1n02x4               g076(.a(new_n168), .b(new_n171), .c(new_n162), .out0(new_n172));
  aoi022aa1n02x5               g077(.a(new_n165), .b(new_n168), .c(new_n146), .d(new_n172), .o1(\s[13] ));
  aoi012aa1n02x5               g078(.a(new_n166), .b(new_n165), .c(new_n167), .o1(new_n174));
  xnrb03aa1n02x5               g079(.a(new_n174), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d24x5               g080(.a(\b[13] ), .b(\a[14] ), .o1(new_n176));
  nanp02aa1n12x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nano23aa1n09x5               g082(.a(new_n166), .b(new_n176), .c(new_n177), .d(new_n167), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n163), .c(new_n124), .d(new_n145), .o1(new_n179));
  oa0012aa1n02x5               g084(.a(new_n177), .b(new_n176), .c(new_n166), .o(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  nor002aa1d32x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand42aa1n06x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n03x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n179), .c(new_n181), .out0(\s[15] ));
  aobi12aa1n06x5               g090(.a(new_n184), .b(new_n179), .c(new_n181), .out0(new_n186));
  nor002aa1d32x5               g091(.a(\b[15] ), .b(\a[16] ), .o1(new_n187));
  nand42aa1n04x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  norb02aa1n06x4               g093(.a(new_n188), .b(new_n187), .out0(new_n189));
  oabi12aa1n03x5               g094(.a(new_n189), .b(new_n186), .c(new_n182), .out0(new_n190));
  inv040aa1n09x5               g095(.a(new_n182), .o1(new_n191));
  nano22aa1n03x7               g096(.a(new_n186), .b(new_n191), .c(new_n189), .out0(new_n192));
  nanb02aa1n03x5               g097(.a(new_n192), .b(new_n190), .out0(\s[16] ));
  nand23aa1n03x5               g098(.a(new_n178), .b(new_n184), .c(new_n189), .o1(new_n194));
  nor042aa1n03x5               g099(.a(new_n194), .b(new_n159), .o1(new_n195));
  nand02aa1d04x5               g100(.a(new_n124), .b(new_n195), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  nona23aa1n09x5               g102(.a(new_n177), .b(new_n167), .c(new_n166), .d(new_n176), .out0(new_n198));
  nona23aa1n03x5               g103(.a(new_n188), .b(new_n183), .c(new_n182), .d(new_n187), .out0(new_n199));
  nor042aa1n03x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  nanb03aa1n03x5               g105(.a(new_n187), .b(new_n188), .c(new_n183), .out0(new_n201));
  oai112aa1n06x5               g106(.a(new_n191), .b(new_n177), .c(new_n176), .d(new_n166), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n197), .b(new_n187), .c(new_n188), .d(new_n182), .o1(new_n203));
  oai012aa1n02x5               g108(.a(new_n203), .b(new_n202), .c(new_n201), .o1(new_n204));
  aoi012aa1n02x5               g109(.a(new_n204), .b(new_n163), .c(new_n200), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(new_n145), .b(new_n200), .o1(new_n206));
  tech160nm_fioaoi03aa1n04x5   g111(.a(\a[16] ), .b(\b[15] ), .c(new_n191), .o1(new_n207));
  oabi12aa1n06x5               g112(.a(new_n207), .b(new_n201), .c(new_n202), .out0(new_n208));
  aoi012aa1d18x5               g113(.a(new_n208), .b(new_n163), .c(new_n200), .o1(new_n209));
  aoai13aa1n09x5               g114(.a(new_n209), .b(new_n206), .c(new_n158), .d(new_n123), .o1(new_n210));
  aoi022aa1n02x5               g115(.a(new_n210), .b(new_n197), .c(new_n196), .d(new_n205), .o1(\s[17] ));
  nor042aa1n03x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\a[17] ), .o1(new_n214));
  oaib12aa1n03x5               g119(.a(new_n210), .b(new_n214), .c(\b[16] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n215), .c(new_n213), .out0(\s[18] ));
  oab012aa1n02x4               g122(.a(new_n207), .b(new_n202), .c(new_n201), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n194), .c(new_n171), .d(new_n162), .o1(new_n219));
  inv040aa1d32x5               g124(.a(\a[18] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n214), .b(\b[16] ), .c(new_n220), .d(\b[17] ), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n219), .c(new_n124), .d(new_n195), .o1(new_n222));
  tech160nm_fioaoi03aa1n03p5x5 g127(.a(\a[18] ), .b(\b[17] ), .c(new_n213), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  nor042aa1d18x5               g129(.a(\b[18] ), .b(\a[19] ), .o1(new_n225));
  nand02aa1n06x5               g130(.a(\b[18] ), .b(\a[19] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n224), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g134(.a(new_n222), .b(new_n224), .o1(new_n230));
  nor042aa1n12x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand02aa1d28x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nanb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n225), .c(new_n230), .d(new_n226), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n227), .b(new_n223), .c(new_n210), .d(new_n221), .o1(new_n235));
  nona22aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n225), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[20] ));
  nano23aa1n06x5               g142(.a(new_n225), .b(new_n231), .c(new_n232), .d(new_n226), .out0(new_n238));
  nand23aa1n02x5               g143(.a(new_n238), .b(new_n197), .c(new_n216), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n219), .c(new_n124), .d(new_n195), .o1(new_n241));
  nanb03aa1n03x5               g146(.a(new_n231), .b(new_n232), .c(new_n226), .out0(new_n242));
  nand02aa1n04x5               g147(.a(\b[17] ), .b(\a[18] ), .o1(new_n243));
  oai022aa1d18x5               g148(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n244));
  oai112aa1n06x5               g149(.a(new_n244), .b(new_n243), .c(\b[18] ), .d(\a[19] ), .o1(new_n245));
  tech160nm_fiaoi012aa1n04x5   g150(.a(new_n231), .b(new_n225), .c(new_n232), .o1(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n246), .b(new_n245), .c(new_n242), .o1(new_n247));
  nanb02aa1n06x5               g152(.a(new_n247), .b(new_n241), .out0(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[20] ), .b(\a[21] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  nano22aa1n03x7               g155(.a(new_n231), .b(new_n226), .c(new_n232), .out0(new_n251));
  oai012aa1n02x5               g156(.a(new_n243), .b(\b[18] ), .c(\a[19] ), .o1(new_n252));
  norb02aa1n02x7               g157(.a(new_n244), .b(new_n252), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n246), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n253), .d(new_n251), .o1(new_n255));
  aoi022aa1n02x5               g160(.a(new_n248), .b(new_n250), .c(new_n241), .d(new_n255), .o1(\s[21] ));
  nor042aa1d18x5               g161(.a(\b[20] ), .b(\a[21] ), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[21] ), .b(\a[22] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n257), .c(new_n248), .d(new_n250), .o1(new_n259));
  aoai13aa1n02x5               g164(.a(new_n250), .b(new_n247), .c(new_n210), .d(new_n240), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n258), .c(new_n257), .out0(new_n261));
  nanp02aa1n03x5               g166(.a(new_n259), .b(new_n261), .o1(\s[22] ));
  nor042aa1n06x5               g167(.a(new_n258), .b(new_n249), .o1(new_n263));
  nand23aa1n03x5               g168(.a(new_n221), .b(new_n263), .c(new_n238), .o1(new_n264));
  inv040aa1n03x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n219), .c(new_n124), .d(new_n195), .o1(new_n266));
  inv000aa1d42x5               g171(.a(\a[22] ), .o1(new_n267));
  inv040aa1d32x5               g172(.a(\b[21] ), .o1(new_n268));
  oao003aa1n06x5               g173(.a(new_n267), .b(new_n268), .c(new_n257), .carry(new_n269));
  aoi012aa1n02x5               g174(.a(new_n269), .b(new_n247), .c(new_n263), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n264), .c(new_n196), .d(new_n209), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[23] ), .b(\b[22] ), .out0(new_n272));
  aoi112aa1n02x5               g177(.a(new_n272), .b(new_n269), .c(new_n247), .d(new_n263), .o1(new_n273));
  aoi022aa1n02x5               g178(.a(new_n271), .b(new_n272), .c(new_n266), .d(new_n273), .o1(\s[23] ));
  norp02aa1n02x5               g179(.a(\b[22] ), .b(\a[23] ), .o1(new_n275));
  xnrc02aa1n12x5               g180(.a(\b[23] ), .b(\a[24] ), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n271), .d(new_n272), .o1(new_n277));
  tech160nm_finand02aa1n03p5x5 g182(.a(new_n271), .b(new_n272), .o1(new_n278));
  nona22aa1n03x5               g183(.a(new_n278), .b(new_n276), .c(new_n275), .out0(new_n279));
  nanp02aa1n03x5               g184(.a(new_n279), .b(new_n277), .o1(\s[24] ));
  inv000aa1d42x5               g185(.a(new_n272), .o1(new_n281));
  nona32aa1n03x5               g186(.a(new_n210), .b(new_n276), .c(new_n281), .d(new_n264), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n263), .b(new_n254), .c(new_n253), .d(new_n251), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n269), .o1(new_n284));
  norb02aa1n06x4               g189(.a(new_n272), .b(new_n276), .out0(new_n285));
  inv000aa1n06x5               g190(.a(new_n285), .o1(new_n286));
  aoi112aa1n02x5               g191(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n287));
  oab012aa1n04x5               g192(.a(new_n287), .b(\a[24] ), .c(\b[23] ), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n286), .c(new_n283), .d(new_n284), .o1(new_n289));
  inv000aa1n02x5               g194(.a(new_n289), .o1(new_n290));
  nanp02aa1n06x5               g195(.a(new_n282), .b(new_n290), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[25] ), .b(\b[24] ), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n285), .b(new_n269), .c(new_n247), .d(new_n263), .o1(new_n293));
  nano22aa1n02x4               g198(.a(new_n292), .b(new_n293), .c(new_n288), .out0(new_n294));
  aoi022aa1n02x5               g199(.a(new_n291), .b(new_n292), .c(new_n282), .d(new_n294), .o1(\s[25] ));
  nor002aa1n02x5               g200(.a(\b[24] ), .b(\a[25] ), .o1(new_n296));
  xnrc02aa1n12x5               g201(.a(\b[25] ), .b(\a[26] ), .out0(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n296), .c(new_n291), .d(new_n292), .o1(new_n298));
  aoi012aa1n06x5               g203(.a(new_n219), .b(new_n124), .c(new_n195), .o1(new_n299));
  nano22aa1n03x7               g204(.a(new_n299), .b(new_n265), .c(new_n285), .out0(new_n300));
  tech160nm_fioai012aa1n05x5   g205(.a(new_n292), .b(new_n300), .c(new_n289), .o1(new_n301));
  nona22aa1n03x5               g206(.a(new_n301), .b(new_n297), .c(new_n296), .out0(new_n302));
  nanp02aa1n03x5               g207(.a(new_n298), .b(new_n302), .o1(\s[26] ));
  norb02aa1n03x5               g208(.a(new_n292), .b(new_n297), .out0(new_n304));
  inv000aa1n02x5               g209(.a(new_n304), .o1(new_n305));
  nona32aa1n09x5               g210(.a(new_n210), .b(new_n305), .c(new_n286), .d(new_n264), .out0(new_n306));
  nor042aa1n06x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  and002aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o(new_n308));
  norp02aa1n06x5               g213(.a(new_n308), .b(new_n307), .o1(new_n309));
  inv000aa1d42x5               g214(.a(\a[26] ), .o1(new_n310));
  inv000aa1d42x5               g215(.a(\b[25] ), .o1(new_n311));
  oao003aa1n02x5               g216(.a(new_n310), .b(new_n311), .c(new_n296), .carry(new_n312));
  aoi112aa1n02x7               g217(.a(new_n312), .b(new_n309), .c(new_n289), .d(new_n304), .o1(new_n313));
  aoi012aa1n06x5               g218(.a(new_n312), .b(new_n289), .c(new_n304), .o1(new_n314));
  nanp02aa1n03x5               g219(.a(new_n314), .b(new_n306), .o1(new_n315));
  aoi022aa1n02x7               g220(.a(new_n315), .b(new_n309), .c(new_n306), .d(new_n313), .o1(\s[27] ));
  nano32aa1n03x7               g221(.a(new_n299), .b(new_n304), .c(new_n265), .d(new_n285), .out0(new_n317));
  inv000aa1n02x5               g222(.a(new_n312), .o1(new_n318));
  aoai13aa1n02x7               g223(.a(new_n318), .b(new_n305), .c(new_n293), .d(new_n288), .o1(new_n319));
  oabi12aa1n02x5               g224(.a(new_n308), .b(new_n317), .c(new_n319), .out0(new_n320));
  inv000aa1d42x5               g225(.a(new_n307), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n308), .c(new_n314), .d(new_n306), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[28] ), .b(\b[27] ), .out0(new_n323));
  norp02aa1n02x5               g228(.a(new_n323), .b(new_n307), .o1(new_n324));
  aoi022aa1n03x5               g229(.a(new_n322), .b(new_n323), .c(new_n320), .d(new_n324), .o1(\s[28] ));
  and002aa1n02x5               g230(.a(new_n323), .b(new_n309), .o(new_n326));
  oai012aa1n03x5               g231(.a(new_n326), .b(new_n317), .c(new_n319), .o1(new_n327));
  inv040aa1n03x5               g232(.a(new_n326), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[28] ), .b(\b[27] ), .c(new_n321), .carry(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n328), .c(new_n314), .d(new_n306), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .out0(new_n331));
  norb02aa1n02x5               g236(.a(new_n329), .b(new_n331), .out0(new_n332));
  aoi022aa1n03x5               g237(.a(new_n330), .b(new_n331), .c(new_n327), .d(new_n332), .o1(\s[29] ));
  xorb03aa1n02x5               g238(.a(new_n151), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g239(.a(new_n323), .b(new_n331), .c(new_n309), .o(new_n335));
  oaih12aa1n02x5               g240(.a(new_n335), .b(new_n317), .c(new_n319), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n335), .o1(new_n337));
  inv000aa1d42x5               g242(.a(\b[28] ), .o1(new_n338));
  inv000aa1d42x5               g243(.a(\a[29] ), .o1(new_n339));
  oaib12aa1n02x5               g244(.a(new_n329), .b(\b[28] ), .c(new_n339), .out0(new_n340));
  oaib12aa1n02x5               g245(.a(new_n340), .b(new_n338), .c(\a[29] ), .out0(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n337), .c(new_n314), .d(new_n306), .o1(new_n342));
  xorc02aa1n02x5               g247(.a(\a[30] ), .b(\b[29] ), .out0(new_n343));
  oaoi13aa1n02x5               g248(.a(new_n343), .b(new_n340), .c(new_n339), .d(new_n338), .o1(new_n344));
  aoi022aa1n03x5               g249(.a(new_n342), .b(new_n343), .c(new_n336), .d(new_n344), .o1(\s[30] ));
  nano22aa1n02x4               g250(.a(new_n328), .b(new_n331), .c(new_n343), .out0(new_n346));
  oaih12aa1n02x5               g251(.a(new_n346), .b(new_n317), .c(new_n319), .o1(new_n347));
  aoi022aa1n02x5               g252(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n348));
  norb02aa1n02x5               g253(.a(\a[31] ), .b(\b[30] ), .out0(new_n349));
  obai22aa1n02x7               g254(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n350));
  aoi112aa1n02x5               g255(.a(new_n350), .b(new_n349), .c(new_n340), .d(new_n348), .o1(new_n351));
  xorc02aa1n02x5               g256(.a(\a[31] ), .b(\b[30] ), .out0(new_n352));
  inv000aa1n02x5               g257(.a(new_n346), .o1(new_n353));
  norp02aa1n02x5               g258(.a(\b[29] ), .b(\a[30] ), .o1(new_n354));
  aoi012aa1n02x5               g259(.a(new_n354), .b(new_n340), .c(new_n348), .o1(new_n355));
  aoai13aa1n03x5               g260(.a(new_n355), .b(new_n353), .c(new_n314), .d(new_n306), .o1(new_n356));
  aoi022aa1n03x5               g261(.a(new_n356), .b(new_n352), .c(new_n347), .d(new_n351), .o1(\s[31] ));
  xnbna2aa1n03x5               g262(.a(new_n100), .b(new_n152), .c(new_n150), .out0(\s[3] ));
  oai012aa1n02x5               g263(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n359));
  xorc02aa1n02x5               g264(.a(\a[4] ), .b(\b[3] ), .out0(new_n360));
  xnbna2aa1n03x5               g265(.a(new_n360), .b(new_n359), .c(new_n148), .out0(\s[4] ));
  norb02aa1n02x5               g266(.a(new_n105), .b(new_n156), .out0(new_n362));
  xobna2aa1n03x5               g267(.a(new_n362), .b(new_n154), .c(new_n111), .out0(\s[5] ));
  aoai13aa1n06x5               g268(.a(new_n362), .b(new_n104), .c(\a[4] ), .d(\b[3] ), .o1(new_n364));
  xorc02aa1n02x5               g269(.a(\a[6] ), .b(\b[5] ), .out0(new_n365));
  xobna2aa1n03x5               g270(.a(new_n365), .b(new_n364), .c(new_n105), .out0(\s[6] ));
  aobi12aa1n06x5               g271(.a(new_n365), .b(new_n364), .c(new_n105), .out0(new_n367));
  norb02aa1n02x5               g272(.a(new_n107), .b(new_n367), .out0(new_n368));
  nano23aa1n06x5               g273(.a(new_n367), .b(new_n113), .c(new_n107), .d(new_n114), .out0(new_n369));
  oab012aa1n02x4               g274(.a(new_n369), .b(new_n368), .c(new_n115), .out0(\s[7] ));
  nanp02aa1n02x5               g275(.a(new_n118), .b(new_n109), .o1(new_n371));
  oai012aa1n02x5               g276(.a(new_n371), .b(new_n369), .c(new_n113), .o1(new_n372));
  orn003aa1n03x5               g277(.a(new_n369), .b(new_n371), .c(new_n113), .o(new_n373));
  nanp02aa1n03x5               g278(.a(new_n373), .b(new_n372), .o1(\s[8] ));
  norb02aa1n02x5               g279(.a(new_n125), .b(new_n97), .out0(new_n375));
  norp03aa1n02x5               g280(.a(new_n121), .b(new_n122), .c(new_n375), .o1(new_n376));
  aoi022aa1n02x5               g281(.a(new_n124), .b(new_n375), .c(new_n158), .d(new_n376), .o1(\s[9] ));
endmodule


