// Benchmark "adder" written by ABC on Thu Jul 18 03:33:27 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n344,
    new_n345, new_n346, new_n348, new_n349, new_n351, new_n353, new_n354,
    new_n355, new_n357, new_n358, new_n359, new_n360, new_n362, new_n363,
    new_n365, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  tech160nm_finand02aa1n05x5   g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nand22aa1n12x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aoi012aa1d24x5               g007(.a(new_n102), .b(\a[2] ), .c(\b[1] ), .o1(new_n103));
  inv040aa1d32x5               g008(.a(\a[3] ), .o1(new_n104));
  aoi022aa1d24x5               g009(.a(\b[2] ), .b(\a[3] ), .c(\a[2] ), .d(\b[1] ), .o1(new_n105));
  oaib12aa1n18x5               g010(.a(new_n105), .b(\b[2] ), .c(new_n104), .out0(new_n106));
  oa0022aa1n12x5               g011(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n107));
  aoai13aa1n12x5               g012(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n101), .o1(new_n108));
  nand02aa1n08x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor002aa1n02x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  aoi022aa1d24x5               g017(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand22aa1n03x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand02aa1d28x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  nano32aa1n06x5               g023(.a(new_n118), .b(new_n112), .c(new_n113), .d(new_n109), .out0(new_n119));
  oaih12aa1n02x5               g024(.a(new_n117), .b(new_n116), .c(new_n111), .o1(new_n120));
  nano22aa1n12x5               g025(.a(new_n111), .b(new_n109), .c(new_n117), .out0(new_n121));
  tech160nm_fiaoi012aa1n05x5   g026(.a(new_n116), .b(\a[6] ), .c(\b[5] ), .o1(new_n122));
  oai012aa1n06x5               g027(.a(new_n122), .b(new_n114), .c(new_n110), .o1(new_n123));
  oaib12aa1n02x5               g028(.a(new_n120), .b(new_n123), .c(new_n121), .out0(new_n124));
  xnrc02aa1n12x5               g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n108), .d(new_n119), .o1(new_n127));
  nor022aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanb02aa1n12x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  nand42aa1n02x5               g037(.a(new_n119), .b(new_n108), .o1(new_n133));
  oai022aa1n02x5               g038(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n134));
  aboi22aa1n06x5               g039(.a(new_n123), .b(new_n121), .c(new_n134), .d(new_n117), .out0(new_n135));
  aoai13aa1n04x5               g040(.a(new_n98), .b(new_n125), .c(new_n133), .d(new_n135), .o1(new_n136));
  oaoi03aa1n02x5               g041(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n137));
  nor042aa1n04x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand02aa1n03x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n137), .c(new_n136), .d(new_n131), .o1(new_n141));
  aoi112aa1n02x5               g046(.a(new_n140), .b(new_n137), .c(new_n136), .d(new_n131), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(\s[11] ));
  tech160nm_fioai012aa1n03p5x5 g048(.a(new_n141), .b(\b[10] ), .c(\a[11] ), .o1(new_n144));
  nor022aa1n08x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand22aa1n09x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  aoib12aa1n02x5               g052(.a(new_n138), .b(new_n146), .c(new_n145), .out0(new_n148));
  aoi022aa1n03x5               g053(.a(new_n144), .b(new_n147), .c(new_n141), .d(new_n148), .o1(\s[12] ));
  nona23aa1n09x5               g054(.a(new_n146), .b(new_n139), .c(new_n138), .d(new_n145), .out0(new_n150));
  nor043aa1d12x5               g055(.a(new_n150), .b(new_n130), .c(new_n125), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n124), .c(new_n108), .d(new_n119), .o1(new_n152));
  aoi012aa1n06x5               g057(.a(new_n124), .b(new_n108), .c(new_n119), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n151), .o1(new_n154));
  nano22aa1n03x7               g059(.a(new_n145), .b(new_n139), .c(new_n146), .out0(new_n155));
  tech160nm_fioai012aa1n03p5x5 g060(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .o1(new_n156));
  oab012aa1n06x5               g061(.a(new_n156), .b(new_n97), .c(new_n128), .out0(new_n157));
  aoi012aa1n02x7               g062(.a(new_n145), .b(new_n138), .c(new_n146), .o1(new_n158));
  aob012aa1n02x5               g063(.a(new_n158), .b(new_n157), .c(new_n155), .out0(new_n159));
  oabi12aa1n09x5               g064(.a(new_n159), .b(new_n153), .c(new_n154), .out0(new_n160));
  nor042aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  inv020aa1n02x5               g068(.a(new_n158), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n157), .d(new_n155), .o1(new_n165));
  aoi022aa1n02x5               g070(.a(new_n160), .b(new_n163), .c(new_n152), .d(new_n165), .o1(\s[13] ));
  nor042aa1n04x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand22aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n161), .c(new_n160), .d(new_n162), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(new_n161), .b(new_n169), .c(new_n160), .d(new_n163), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[14] ));
  nano23aa1n06x5               g077(.a(new_n161), .b(new_n167), .c(new_n168), .d(new_n162), .out0(new_n173));
  oaih12aa1n06x5               g078(.a(new_n168), .b(new_n167), .c(new_n161), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nor002aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanp02aa1n03x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n175), .c(new_n160), .d(new_n173), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n179), .b(new_n175), .c(new_n160), .d(new_n173), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(\s[15] ));
  oai012aa1n02x5               g087(.a(new_n180), .b(\b[14] ), .c(\a[15] ), .o1(new_n183));
  nor042aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nand02aa1d04x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  nanb02aa1n02x5               g090(.a(new_n184), .b(new_n185), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  aoib12aa1n02x5               g092(.a(new_n176), .b(new_n185), .c(new_n184), .out0(new_n188));
  aoi022aa1n02x5               g093(.a(new_n183), .b(new_n187), .c(new_n180), .d(new_n188), .o1(\s[16] ));
  nona23aa1n02x4               g094(.a(new_n168), .b(new_n162), .c(new_n161), .d(new_n167), .out0(new_n190));
  nona23aa1d18x5               g095(.a(new_n185), .b(new_n177), .c(new_n176), .d(new_n184), .out0(new_n191));
  nona22aa1n09x5               g096(.a(new_n151), .b(new_n190), .c(new_n191), .out0(new_n192));
  aoi012aa1n12x5               g097(.a(new_n192), .b(new_n133), .c(new_n135), .o1(new_n193));
  aoai13aa1n04x5               g098(.a(new_n173), .b(new_n164), .c(new_n157), .d(new_n155), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n184), .b(new_n176), .c(new_n185), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n191), .c(new_n194), .d(new_n174), .o1(new_n196));
  xorc02aa1n12x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  tech160nm_fioai012aa1n03p5x5 g102(.a(new_n197), .b(new_n196), .c(new_n193), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n191), .o1(new_n199));
  aob012aa1n06x5               g104(.a(new_n199), .b(new_n194), .c(new_n174), .out0(new_n200));
  nano23aa1n02x4               g105(.a(new_n197), .b(new_n193), .c(new_n200), .d(new_n195), .out0(new_n201));
  norb02aa1n02x5               g106(.a(new_n198), .b(new_n201), .out0(\s[17] ));
  nor002aa1d32x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  nor042aa1n09x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n03x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n198), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g113(.a(new_n197), .b(new_n207), .o(new_n209));
  tech160nm_fioai012aa1n03p5x5 g114(.a(new_n209), .b(new_n196), .c(new_n193), .o1(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nor042aa1d18x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand02aa1n08x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n12x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n210), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oai112aa1n06x5               g122(.a(new_n200), .b(new_n195), .c(new_n192), .d(new_n153), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n215), .b(new_n211), .c(new_n218), .d(new_n209), .o1(new_n219));
  inv040aa1n03x5               g124(.a(new_n213), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n215), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n221), .c(new_n210), .d(new_n212), .o1(new_n222));
  nor042aa1n06x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand02aa1d28x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  inv000aa1d42x5               g130(.a(\a[19] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[18] ), .o1(new_n227));
  aboi22aa1n03x5               g132(.a(new_n223), .b(new_n224), .c(new_n226), .d(new_n227), .out0(new_n228));
  aoi022aa1n03x5               g133(.a(new_n222), .b(new_n225), .c(new_n219), .d(new_n228), .o1(\s[20] ));
  nano32aa1n03x7               g134(.a(new_n221), .b(new_n197), .c(new_n225), .d(new_n207), .out0(new_n230));
  oaih12aa1n02x5               g135(.a(new_n230), .b(new_n196), .c(new_n193), .o1(new_n231));
  nanb03aa1n09x5               g136(.a(new_n223), .b(new_n224), .c(new_n214), .out0(new_n232));
  oai112aa1n06x5               g137(.a(new_n220), .b(new_n206), .c(new_n205), .d(new_n203), .o1(new_n233));
  aoi012aa1n06x5               g138(.a(new_n223), .b(new_n213), .c(new_n224), .o1(new_n234));
  oai012aa1n18x5               g139(.a(new_n234), .b(new_n233), .c(new_n232), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n235), .c(new_n218), .d(new_n230), .o1(new_n238));
  nano22aa1n02x4               g143(.a(new_n223), .b(new_n214), .c(new_n224), .out0(new_n239));
  oai012aa1n02x5               g144(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(new_n203), .c(new_n205), .out0(new_n241));
  inv020aa1n02x5               g146(.a(new_n234), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n237), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n243));
  aobi12aa1n02x7               g148(.a(new_n238), .b(new_n243), .c(new_n231), .out0(\s[21] ));
  inv000aa1d42x5               g149(.a(new_n235), .o1(new_n245));
  nor042aa1n03x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  inv040aa1n03x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n02x7               g152(.a(new_n247), .b(new_n236), .c(new_n231), .d(new_n245), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[21] ), .b(\a[22] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n246), .out0(new_n251));
  aoi022aa1n03x5               g156(.a(new_n248), .b(new_n250), .c(new_n238), .d(new_n251), .o1(\s[22] ));
  nor042aa1n06x5               g157(.a(new_n249), .b(new_n236), .o1(new_n253));
  and002aa1n02x5               g158(.a(new_n230), .b(new_n253), .o(new_n254));
  oaih12aa1n02x5               g159(.a(new_n254), .b(new_n196), .c(new_n193), .o1(new_n255));
  oao003aa1n12x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .carry(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(new_n235), .c(new_n253), .o1(new_n258));
  inv040aa1n03x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n218), .d(new_n254), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n257), .c(new_n235), .d(new_n253), .o1(new_n262));
  aobi12aa1n03x7               g167(.a(new_n261), .b(new_n262), .c(new_n255), .out0(\s[23] ));
  nor042aa1n06x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n260), .o1(new_n266));
  aoai13aa1n02x7               g171(.a(new_n265), .b(new_n266), .c(new_n255), .d(new_n258), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n264), .o1(new_n269));
  aoi022aa1n03x5               g174(.a(new_n267), .b(new_n268), .c(new_n261), .d(new_n269), .o1(\s[24] ));
  inv000aa1n02x5               g175(.a(new_n230), .o1(new_n271));
  and002aa1n06x5               g176(.a(new_n268), .b(new_n260), .o(new_n272));
  nano22aa1n06x5               g177(.a(new_n271), .b(new_n272), .c(new_n253), .out0(new_n273));
  oaih12aa1n02x5               g178(.a(new_n273), .b(new_n196), .c(new_n193), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n253), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n275));
  inv020aa1n03x5               g180(.a(new_n272), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .carry(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .d(new_n256), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n278), .c(new_n218), .d(new_n273), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n272), .b(new_n257), .c(new_n235), .d(new_n253), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n279), .o1(new_n282));
  and003aa1n02x5               g187(.a(new_n281), .b(new_n282), .c(new_n277), .o(new_n283));
  aobi12aa1n03x7               g188(.a(new_n280), .b(new_n283), .c(new_n274), .out0(\s[25] ));
  inv000aa1d42x5               g189(.a(new_n278), .o1(new_n285));
  nor042aa1n06x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  aoai13aa1n02x7               g192(.a(new_n287), .b(new_n282), .c(new_n274), .d(new_n285), .o1(new_n288));
  tech160nm_fixorc02aa1n03p5x5 g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n286), .o1(new_n290));
  aoi022aa1n03x5               g195(.a(new_n288), .b(new_n289), .c(new_n280), .d(new_n290), .o1(\s[26] ));
  and002aa1n12x5               g196(.a(new_n289), .b(new_n279), .o(new_n292));
  nano32aa1n06x5               g197(.a(new_n271), .b(new_n292), .c(new_n253), .d(new_n272), .out0(new_n293));
  tech160nm_fioai012aa1n05x5   g198(.a(new_n293), .b(new_n196), .c(new_n193), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n292), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n296));
  aoai13aa1n04x5               g201(.a(new_n296), .b(new_n295), .c(new_n281), .d(new_n277), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n218), .d(new_n293), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n296), .o1(new_n300));
  aoi112aa1n02x5               g205(.a(new_n298), .b(new_n300), .c(new_n278), .d(new_n292), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n299), .b(new_n301), .c(new_n294), .out0(\s[27] ));
  aoi012aa1n12x5               g207(.a(new_n300), .b(new_n278), .c(new_n292), .o1(new_n303));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  inv000aa1n03x5               g209(.a(new_n304), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n298), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n306), .c(new_n303), .d(new_n294), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n304), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n299), .d(new_n309), .o1(\s[28] ));
  and002aa1n02x5               g215(.a(new_n308), .b(new_n298), .o(new_n311));
  aoai13aa1n02x5               g216(.a(new_n311), .b(new_n297), .c(new_n218), .d(new_n293), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n313), .c(new_n303), .d(new_n294), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n314), .b(new_n316), .out0(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g224(.a(new_n306), .b(new_n308), .c(new_n316), .out0(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n297), .c(new_n218), .d(new_n293), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\b[28] ), .o1(new_n323));
  inv000aa1d42x5               g228(.a(\a[29] ), .o1(new_n324));
  oaib12aa1n02x5               g229(.a(new_n314), .b(\b[28] ), .c(new_n324), .out0(new_n325));
  oaib12aa1n02x5               g230(.a(new_n325), .b(new_n323), .c(\a[29] ), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n322), .c(new_n303), .d(new_n294), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  oaoi13aa1n02x5               g233(.a(new_n328), .b(new_n325), .c(new_n324), .d(new_n323), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n321), .d(new_n329), .o1(\s[30] ));
  nanb02aa1n02x5               g235(.a(\b[30] ), .b(\a[31] ), .out0(new_n331));
  nanb02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  nanp02aa1n02x5               g237(.a(new_n332), .b(new_n331), .o1(new_n333));
  nano32aa1n03x7               g238(.a(new_n306), .b(new_n328), .c(new_n308), .d(new_n316), .out0(new_n334));
  aoai13aa1n02x5               g239(.a(new_n334), .b(new_n297), .c(new_n218), .d(new_n293), .o1(new_n335));
  inv000aa1d42x5               g240(.a(new_n334), .o1(new_n336));
  norp02aa1n02x5               g241(.a(\b[29] ), .b(\a[30] ), .o1(new_n337));
  aoi022aa1n02x5               g242(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n337), .b(new_n325), .c(new_n338), .o1(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n336), .c(new_n303), .d(new_n294), .o1(new_n340));
  oai112aa1n02x5               g245(.a(new_n331), .b(new_n332), .c(\b[29] ), .d(\a[30] ), .o1(new_n341));
  aoi012aa1n02x5               g246(.a(new_n341), .b(new_n325), .c(new_n338), .o1(new_n342));
  aoi022aa1n03x5               g247(.a(new_n340), .b(new_n333), .c(new_n335), .d(new_n342), .o1(\s[31] ));
  aoi012aa1n02x5               g248(.a(new_n106), .b(new_n101), .c(new_n103), .o1(new_n344));
  xnrc02aa1n02x5               g249(.a(\b[2] ), .b(\a[3] ), .out0(new_n345));
  oaoi03aa1n02x5               g250(.a(new_n99), .b(new_n100), .c(new_n102), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n344), .b(new_n345), .c(new_n346), .o1(\s[3] ));
  xorc02aa1n02x5               g252(.a(\a[4] ), .b(\b[3] ), .out0(new_n348));
  aoib12aa1n02x5               g253(.a(new_n348), .b(new_n104), .c(\b[2] ), .out0(new_n349));
  aboi22aa1n03x5               g254(.a(new_n344), .b(new_n349), .c(new_n108), .d(new_n348), .out0(\s[4] ));
  xorc02aa1n02x5               g255(.a(\a[5] ), .b(\b[4] ), .out0(new_n351));
  xobna2aa1n03x5               g256(.a(new_n351), .b(new_n108), .c(new_n115), .out0(\s[5] ));
  inv000aa1d42x5               g257(.a(new_n114), .o1(new_n353));
  nanp03aa1n02x5               g258(.a(new_n108), .b(new_n115), .c(new_n351), .o1(new_n354));
  xorc02aa1n02x5               g259(.a(\a[6] ), .b(\b[5] ), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n354), .c(new_n353), .out0(\s[6] ));
  nanp02aa1n02x5               g261(.a(new_n354), .b(new_n353), .o1(new_n357));
  norb02aa1n02x5               g262(.a(new_n109), .b(new_n111), .out0(new_n358));
  aoai13aa1n02x5               g263(.a(new_n358), .b(new_n110), .c(new_n357), .d(new_n355), .o1(new_n359));
  aoi112aa1n02x5               g264(.a(new_n358), .b(new_n110), .c(new_n357), .d(new_n355), .o1(new_n360));
  norb02aa1n02x5               g265(.a(new_n359), .b(new_n360), .out0(\s[7] ));
  inv000aa1d42x5               g266(.a(new_n111), .o1(new_n362));
  norb02aa1n02x5               g267(.a(new_n117), .b(new_n116), .out0(new_n363));
  xnbna2aa1n03x5               g268(.a(new_n363), .b(new_n359), .c(new_n362), .out0(\s[8] ));
  nanp02aa1n02x5               g269(.a(new_n125), .b(new_n120), .o1(new_n365));
  aoib12aa1n02x5               g270(.a(new_n365), .b(new_n121), .c(new_n123), .out0(new_n366));
  aboi22aa1n03x5               g271(.a(new_n153), .b(new_n126), .c(new_n366), .d(new_n133), .out0(\s[9] ));
endmodule


