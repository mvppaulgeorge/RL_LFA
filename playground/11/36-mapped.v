// Benchmark "adder" written by ABC on Wed Jul 17 17:54:46 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n323, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n349, new_n351, new_n353, new_n354,
    new_n356, new_n357, new_n359, new_n360, new_n361, new_n362, new_n364,
    new_n365, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  oai112aa1n06x5               g001(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  xorc02aa1n12x5               g003(.a(\a[3] ), .b(\b[2] ), .out0(new_n99));
  nanp03aa1n02x5               g004(.a(new_n99), .b(new_n97), .c(new_n98), .o1(new_n100));
  oa0022aa1n12x5               g005(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n101));
  nanp02aa1n06x5               g006(.a(new_n100), .b(new_n101), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n03x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  norp02aa1n04x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nanb03aa1n06x5               g010(.a(new_n105), .b(new_n103), .c(new_n104), .out0(new_n106));
  aoi022aa1n12x5               g011(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n107));
  oai022aa1n12x5               g012(.a(\a[6] ), .b(\b[5] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(new_n108), .o1(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .out0(new_n110));
  nano23aa1n06x5               g015(.a(new_n106), .b(new_n110), .c(new_n109), .d(new_n107), .out0(new_n111));
  inv000aa1d42x5               g016(.a(\a[8] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(new_n113), .b(new_n112), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\a[7] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\b[6] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(new_n116), .b(new_n115), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  oaih12aa1n02x5               g024(.a(new_n118), .b(new_n119), .c(new_n105), .o1(new_n120));
  oai022aa1n02x5               g025(.a(new_n115), .b(new_n116), .c(new_n113), .d(new_n112), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n114), .b(new_n121), .c(new_n120), .d(new_n117), .o1(new_n122));
  xorc02aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n102), .d(new_n111), .o1(new_n124));
  oa0012aa1n02x5               g029(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .o(new_n125));
  xorc02aa1n02x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  nand42aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  oai022aa1d24x5               g032(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  obai22aa1n02x7               g034(.a(new_n124), .b(new_n129), .c(new_n125), .d(new_n126), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(\a[10] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\b[8] ), .o1(new_n132));
  xroi22aa1d04x5               g037(.a(new_n131), .b(\b[9] ), .c(new_n132), .d(\a[9] ), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n122), .c(new_n102), .d(new_n111), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n128), .b(new_n127), .o1(new_n135));
  xorc02aa1n03x5               g040(.a(\a[11] ), .b(\b[10] ), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv040aa1d32x5               g042(.a(\a[11] ), .o1(new_n138));
  inv040aa1d32x5               g043(.a(\b[10] ), .o1(new_n139));
  nand42aa1n02x5               g044(.a(new_n139), .b(new_n138), .o1(new_n140));
  aob012aa1n02x5               g045(.a(new_n136), .b(new_n134), .c(new_n135), .out0(new_n141));
  xorc02aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .out0(new_n142));
  nanp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  oai022aa1n02x5               g048(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n144));
  nanb03aa1n02x5               g049(.a(new_n144), .b(new_n141), .c(new_n143), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n142), .c(new_n140), .d(new_n141), .o1(\s[12] ));
  and003aa1n02x5               g051(.a(new_n133), .b(new_n142), .c(new_n136), .o(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n122), .c(new_n102), .d(new_n111), .o1(new_n148));
  orn002aa1n02x7               g053(.a(\a[12] ), .b(\b[11] ), .o(new_n149));
  oai112aa1n03x5               g054(.a(new_n149), .b(new_n143), .c(new_n139), .d(new_n138), .o1(new_n150));
  nano32aa1n03x7               g055(.a(new_n150), .b(new_n128), .c(new_n140), .d(new_n127), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n144), .b(new_n143), .o1(new_n152));
  norb02aa1n03x4               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  nand42aa1n03x5               g058(.a(new_n148), .b(new_n153), .o1(new_n154));
  tech160nm_fixnrc02aa1n04x5   g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  nano22aa1n02x4               g060(.a(new_n151), .b(new_n152), .c(new_n155), .out0(new_n156));
  aboi22aa1n03x5               g061(.a(new_n155), .b(new_n154), .c(new_n156), .d(new_n148), .out0(\s[13] ));
  aoi012aa1n02x5               g062(.a(new_n155), .b(new_n148), .c(new_n153), .o1(new_n158));
  nor022aa1n16x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1n10x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanb02aa1d36x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nor042aa1n04x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoib12aa1n02x5               g068(.a(new_n163), .b(new_n154), .c(new_n155), .out0(new_n164));
  nona22aa1n02x4               g069(.a(new_n160), .b(new_n159), .c(new_n163), .out0(new_n165));
  oai022aa1n02x5               g070(.a(new_n164), .b(new_n162), .c(new_n165), .d(new_n158), .o1(\s[14] ));
  norp02aa1n02x5               g071(.a(new_n155), .b(new_n161), .o1(new_n167));
  oai012aa1n18x5               g072(.a(new_n160), .b(new_n159), .c(new_n163), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  xorc02aa1n12x5               g074(.a(\a[15] ), .b(\b[14] ), .out0(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n169), .c(new_n154), .d(new_n167), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n170), .b(new_n169), .c(new_n154), .d(new_n167), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  inv000aa1d42x5               g078(.a(\a[15] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[14] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  and002aa1n02x7               g082(.a(\b[15] ), .b(\a[16] ), .o(new_n178));
  nor042aa1n02x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n177), .c(new_n174), .d(new_n175), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n171), .b(new_n180), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n179), .c(new_n171), .d(new_n176), .o1(\s[16] ));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n101), .o1(new_n184));
  aoi013aa1n06x4               g089(.a(new_n184), .b(new_n99), .c(new_n98), .d(new_n97), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n107), .o1(new_n186));
  and002aa1n02x5               g091(.a(\b[6] ), .b(\a[7] ), .o(new_n187));
  norb03aa1n02x5               g092(.a(new_n117), .b(new_n108), .c(new_n187), .out0(new_n188));
  nona22aa1n03x5               g093(.a(new_n188), .b(new_n106), .c(new_n186), .out0(new_n189));
  oabi12aa1n18x5               g094(.a(new_n122), .b(new_n185), .c(new_n189), .out0(new_n190));
  nona23aa1n09x5               g095(.a(new_n179), .b(new_n170), .c(new_n155), .d(new_n161), .out0(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n133), .c(new_n136), .d(new_n142), .out0(new_n192));
  nand02aa1d04x5               g097(.a(new_n190), .b(new_n192), .o1(new_n193));
  ao0022aa1n03x5               g098(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o(new_n194));
  tech160nm_fiaoi012aa1n05x5   g099(.a(new_n194), .b(new_n168), .c(new_n176), .o1(new_n195));
  oabi12aa1n02x5               g100(.a(new_n195), .b(\a[16] ), .c(\b[15] ), .out0(new_n196));
  oab012aa1n02x4               g101(.a(new_n196), .b(new_n153), .c(new_n191), .out0(new_n197));
  inv040aa1d32x5               g102(.a(\a[17] ), .o1(new_n198));
  inv040aa1d28x5               g103(.a(\b[16] ), .o1(new_n199));
  nand02aa1d24x5               g104(.a(new_n199), .b(new_n198), .o1(new_n200));
  and002aa1n02x5               g105(.a(\b[16] ), .b(\a[17] ), .o(new_n201));
  nona22aa1n02x4               g106(.a(new_n200), .b(new_n201), .c(new_n177), .out0(new_n202));
  norp02aa1n02x5               g107(.a(new_n195), .b(new_n202), .o1(new_n203));
  oai112aa1n02x5               g108(.a(new_n193), .b(new_n203), .c(new_n191), .d(new_n153), .o1(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n183), .c(new_n197), .d(new_n193), .o1(\s[17] ));
  nanp02aa1n06x5               g110(.a(new_n193), .b(new_n197), .o1(new_n206));
  oaib12aa1n02x5               g111(.a(new_n206), .b(new_n199), .c(\a[17] ), .out0(new_n207));
  aoai13aa1n02x5               g112(.a(new_n200), .b(new_n201), .c(new_n193), .d(new_n197), .o1(new_n208));
  tech160nm_fixorc02aa1n02p5x5 g113(.a(\a[18] ), .b(\b[17] ), .out0(new_n209));
  norb02aa1n02x5               g114(.a(new_n200), .b(new_n209), .out0(new_n210));
  aoi022aa1n02x5               g115(.a(new_n208), .b(new_n209), .c(new_n207), .d(new_n210), .o1(\s[18] ));
  nor002aa1n02x5               g116(.a(new_n195), .b(new_n177), .o1(new_n212));
  oai012aa1n03x5               g117(.a(new_n212), .b(new_n153), .c(new_n191), .o1(new_n213));
  and002aa1n02x5               g118(.a(new_n209), .b(new_n183), .o(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n213), .c(new_n190), .d(new_n192), .o1(new_n215));
  oaoi03aa1n12x5               g120(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nor002aa1d32x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand42aa1n02x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n215), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n04x5               g127(.a(new_n218), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n220), .b(new_n216), .c(new_n206), .d(new_n214), .o1(new_n224));
  nor042aa1n02x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanp02aa1n04x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n220), .o1(new_n229));
  norb03aa1n02x5               g134(.a(new_n226), .b(new_n218), .c(new_n225), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n229), .c(new_n215), .d(new_n217), .o1(new_n231));
  aoai13aa1n02x5               g136(.a(new_n231), .b(new_n228), .c(new_n224), .d(new_n223), .o1(\s[20] ));
  nano23aa1n06x5               g137(.a(new_n218), .b(new_n225), .c(new_n226), .d(new_n219), .out0(new_n233));
  nand23aa1n06x5               g138(.a(new_n233), .b(new_n183), .c(new_n209), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n213), .c(new_n190), .d(new_n192), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n233), .b(new_n216), .o1(new_n237));
  oaoi03aa1n12x5               g142(.a(\a[20] ), .b(\b[19] ), .c(new_n223), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n237), .b(new_n239), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[20] ), .b(\a[21] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n240), .c(new_n206), .d(new_n235), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n238), .b(new_n242), .c(new_n233), .d(new_n216), .o1(new_n244));
  aobi12aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n236), .out0(\s[21] ));
  norp02aa1n02x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  xnrc02aa1n12x5               g152(.a(\b[21] ), .b(\a[22] ), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n240), .o1(new_n250));
  oai022aa1n02x5               g155(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(\a[22] ), .c(\b[21] ), .o1(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n241), .c(new_n236), .d(new_n250), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n243), .d(new_n247), .o1(\s[22] ));
  nor022aa1n06x5               g159(.a(new_n248), .b(new_n241), .o1(new_n255));
  norb02aa1n09x5               g160(.a(new_n255), .b(new_n234), .out0(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n213), .c(new_n190), .d(new_n192), .o1(new_n257));
  aoai13aa1n12x5               g162(.a(new_n255), .b(new_n238), .c(new_n233), .d(new_n216), .o1(new_n258));
  aob012aa1n02x5               g163(.a(new_n251), .b(\b[21] ), .c(\a[22] ), .out0(new_n259));
  nand02aa1d06x5               g164(.a(new_n258), .b(new_n259), .o1(new_n260));
  xorc02aa1n12x5               g165(.a(\a[23] ), .b(\b[22] ), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n260), .c(new_n206), .d(new_n256), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n261), .o1(new_n263));
  and003aa1n02x5               g168(.a(new_n258), .b(new_n263), .c(new_n259), .o(new_n264));
  aobi12aa1n02x5               g169(.a(new_n262), .b(new_n264), .c(new_n257), .out0(\s[23] ));
  norp02aa1n02x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n260), .o1(new_n269));
  oai022aa1n02x5               g174(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n270), .b(\a[24] ), .c(\b[23] ), .o1(new_n271));
  aoai13aa1n02x7               g176(.a(new_n271), .b(new_n263), .c(new_n257), .d(new_n269), .o1(new_n272));
  aoai13aa1n02x5               g177(.a(new_n272), .b(new_n268), .c(new_n262), .d(new_n267), .o1(\s[24] ));
  nano32aa1n02x5               g178(.a(new_n234), .b(new_n268), .c(new_n255), .d(new_n261), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n213), .c(new_n190), .d(new_n192), .o1(new_n275));
  and002aa1n02x7               g180(.a(new_n268), .b(new_n261), .o(new_n276));
  inv030aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  aob012aa1n02x5               g182(.a(new_n270), .b(\b[23] ), .c(\a[24] ), .out0(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n277), .c(new_n258), .d(new_n259), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n206), .d(new_n274), .o1(new_n281));
  nanb02aa1n02x5               g186(.a(new_n280), .b(new_n278), .out0(new_n282));
  aoi012aa1n02x5               g187(.a(new_n282), .b(new_n260), .c(new_n276), .o1(new_n283));
  aobi12aa1n02x5               g188(.a(new_n281), .b(new_n283), .c(new_n275), .out0(\s[25] ));
  norp02aa1n02x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[26] ), .b(\b[25] ), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n279), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n280), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(\b[25] ), .b(\a[26] ), .o1(new_n290));
  oai022aa1n02x5               g195(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n291));
  norb02aa1n02x5               g196(.a(new_n290), .b(new_n291), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n289), .c(new_n275), .d(new_n288), .o1(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n287), .c(new_n281), .d(new_n286), .o1(\s[26] ));
  and002aa1n02x5               g199(.a(new_n287), .b(new_n280), .o(new_n295));
  nano23aa1n03x7               g200(.a(new_n277), .b(new_n234), .c(new_n295), .d(new_n255), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n213), .c(new_n190), .d(new_n192), .o1(new_n297));
  nor042aa1d18x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  and002aa1n24x5               g203(.a(\b[26] ), .b(\a[27] ), .o(new_n299));
  nor042aa1n04x5               g204(.a(new_n299), .b(new_n298), .o1(new_n300));
  aoi122aa1n02x5               g205(.a(new_n300), .b(new_n290), .c(new_n291), .d(new_n279), .e(new_n295), .o1(new_n301));
  nanp02aa1n06x5               g206(.a(new_n279), .b(new_n295), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n291), .b(new_n290), .o1(new_n303));
  nanp03aa1n06x5               g208(.a(new_n297), .b(new_n302), .c(new_n303), .o1(new_n304));
  aoi022aa1n02x5               g209(.a(new_n297), .b(new_n301), .c(new_n304), .d(new_n300), .o1(\s[27] ));
  inv000aa1d42x5               g210(.a(new_n299), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(new_n307));
  inv000aa1n06x5               g212(.a(new_n298), .o1(new_n308));
  aoi022aa1n12x5               g213(.a(new_n279), .b(new_n295), .c(new_n290), .d(new_n291), .o1(new_n309));
  aoai13aa1n02x7               g214(.a(new_n308), .b(new_n299), .c(new_n309), .d(new_n297), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n298), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n307), .d(new_n312), .o1(\s[28] ));
  inv040aa1n03x5               g218(.a(new_n300), .o1(new_n314));
  norb02aa1n02x5               g219(.a(new_n311), .b(new_n314), .out0(new_n315));
  nanp02aa1n03x5               g220(.a(new_n304), .b(new_n315), .o1(new_n316));
  inv000aa1n02x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n03x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n318));
  aoai13aa1n02x7               g223(.a(new_n318), .b(new_n317), .c(new_n309), .d(new_n297), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[29] ));
  nanp02aa1n02x5               g227(.a(\b[0] ), .b(\a[1] ), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g229(.a(new_n311), .b(new_n320), .c(new_n300), .o(new_n325));
  nanp02aa1n03x5               g230(.a(new_n304), .b(new_n325), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n325), .o1(new_n327));
  oaoi03aa1n02x5               g232(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n328), .o1(new_n329));
  aoai13aa1n02x7               g234(.a(new_n329), .b(new_n327), .c(new_n309), .d(new_n297), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .out0(new_n331));
  and002aa1n02x5               g236(.a(\b[28] ), .b(\a[29] ), .o(new_n332));
  oabi12aa1n02x5               g237(.a(new_n331), .b(\a[29] ), .c(\b[28] ), .out0(new_n333));
  oab012aa1n02x4               g238(.a(new_n333), .b(new_n318), .c(new_n332), .out0(new_n334));
  aoi022aa1n03x5               g239(.a(new_n330), .b(new_n331), .c(new_n326), .d(new_n334), .o1(\s[30] ));
  nano32aa1n02x4               g240(.a(new_n314), .b(new_n331), .c(new_n311), .d(new_n320), .out0(new_n336));
  nanp02aa1n03x5               g241(.a(new_n304), .b(new_n336), .o1(new_n337));
  xorc02aa1n02x5               g242(.a(\a[31] ), .b(\b[30] ), .out0(new_n338));
  inv000aa1d42x5               g243(.a(\a[30] ), .o1(new_n339));
  inv000aa1d42x5               g244(.a(\b[29] ), .o1(new_n340));
  oabi12aa1n02x5               g245(.a(new_n338), .b(\a[30] ), .c(\b[29] ), .out0(new_n341));
  oaoi13aa1n04x5               g246(.a(new_n341), .b(new_n328), .c(new_n339), .d(new_n340), .o1(new_n342));
  inv000aa1n02x5               g247(.a(new_n336), .o1(new_n343));
  oaoi03aa1n02x5               g248(.a(new_n339), .b(new_n340), .c(new_n328), .o1(new_n344));
  aoai13aa1n02x7               g249(.a(new_n344), .b(new_n343), .c(new_n309), .d(new_n297), .o1(new_n345));
  aoi022aa1n03x5               g250(.a(new_n345), .b(new_n338), .c(new_n337), .d(new_n342), .o1(\s[31] ));
  xobna2aa1n03x5               g251(.a(new_n99), .b(new_n98), .c(new_n97), .out0(\s[3] ));
  xorc02aa1n02x5               g252(.a(\a[4] ), .b(\b[3] ), .out0(new_n348));
  oab012aa1n02x4               g253(.a(new_n348), .b(\a[3] ), .c(\b[2] ), .out0(new_n349));
  aoi022aa1n02x5               g254(.a(new_n102), .b(new_n348), .c(new_n100), .d(new_n349), .o1(\s[4] ));
  norb02aa1n02x5               g255(.a(new_n104), .b(new_n105), .out0(new_n351));
  xobna2aa1n03x5               g256(.a(new_n351), .b(new_n102), .c(new_n103), .out0(\s[5] ));
  aoai13aa1n02x5               g257(.a(new_n351), .b(new_n185), .c(\a[4] ), .d(\b[3] ), .o1(new_n353));
  norb02aa1n02x5               g258(.a(new_n118), .b(new_n119), .out0(new_n354));
  xobna2aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n104), .out0(\s[6] ));
  inv000aa1d42x5               g260(.a(new_n119), .o1(new_n356));
  nanp03aa1n02x5               g261(.a(new_n353), .b(new_n104), .c(new_n354), .o1(new_n357));
  xobna2aa1n03x5               g262(.a(new_n110), .b(new_n357), .c(new_n356), .out0(\s[7] ));
  xnrc02aa1n02x5               g263(.a(\b[7] ), .b(\a[8] ), .out0(new_n359));
  aoai13aa1n02x5               g264(.a(new_n117), .b(new_n187), .c(new_n357), .d(new_n356), .o1(new_n360));
  norb02aa1n02x5               g265(.a(new_n117), .b(new_n359), .out0(new_n361));
  aoai13aa1n02x5               g266(.a(new_n361), .b(new_n110), .c(new_n357), .d(new_n356), .o1(new_n362));
  aob012aa1n02x5               g267(.a(new_n362), .b(new_n360), .c(new_n359), .out0(\s[8] ));
  nanp02aa1n02x5               g268(.a(new_n102), .b(new_n111), .o1(new_n364));
  aoi012aa1n02x5               g269(.a(new_n121), .b(new_n120), .c(new_n117), .o1(new_n365));
  aoi112aa1n02x5               g270(.a(new_n365), .b(new_n123), .c(new_n112), .d(new_n113), .o1(new_n366));
  aoi022aa1n02x5               g271(.a(new_n190), .b(new_n123), .c(new_n364), .d(new_n366), .o1(\s[9] ));
endmodule


