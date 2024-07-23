// Benchmark "adder" written by ABC on Thu Jul 18 11:51:17 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n338,
    new_n339, new_n340, new_n342, new_n343, new_n344, new_n345, new_n346,
    new_n348, new_n349, new_n351, new_n352, new_n354, new_n355, new_n356,
    new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  oa0022aa1n02x5               g002(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n98));
  xnrc02aa1n02x5               g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  nand42aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n09x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oai012aa1n09x5               g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  tech160nm_fioai012aa1n05x5   g008(.a(new_n98), .b(new_n99), .c(new_n103), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor042aa1n04x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  aoi012aa1n02x5               g011(.a(new_n106), .b(\a[5] ), .c(\b[4] ), .o1(new_n107));
  nor042aa1n06x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n108), .b(\a[4] ), .c(\b[3] ), .o1(new_n109));
  nand22aa1n03x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand42aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n112), .b(new_n110), .c(new_n113), .d(new_n111), .out0(new_n114));
  nano32aa1n03x7               g019(.a(new_n114), .b(new_n109), .c(new_n107), .d(new_n105), .out0(new_n115));
  nand02aa1d06x5               g020(.a(new_n115), .b(new_n104), .o1(new_n116));
  nanb03aa1n03x5               g021(.a(new_n106), .b(new_n112), .c(new_n105), .out0(new_n117));
  inv040aa1n03x5               g022(.a(new_n108), .o1(new_n118));
  oai112aa1n03x5               g023(.a(new_n118), .b(new_n110), .c(new_n111), .d(new_n113), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n118), .o1(new_n120));
  oab012aa1n04x5               g025(.a(new_n120), .b(new_n119), .c(new_n117), .out0(new_n121));
  nand22aa1n03x5               g026(.a(new_n116), .b(new_n121), .o1(new_n122));
  xnrc02aa1n12x5               g027(.a(\b[8] ), .b(\a[9] ), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n123), .o1(new_n124));
  nor022aa1n16x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n16x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n12x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n97), .c(new_n122), .d(new_n124), .o1(new_n129));
  aoi112aa1n02x5               g034(.a(new_n128), .b(new_n97), .c(new_n122), .d(new_n124), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(\s[10] ));
  oai012aa1n02x5               g036(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand02aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n09x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n129), .c(new_n132), .out0(\s[11] ));
  aob012aa1n03x5               g041(.a(new_n135), .b(new_n129), .c(new_n132), .out0(new_n137));
  nor002aa1d32x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n138), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n133), .b(new_n141), .c(new_n139), .o1(new_n142));
  inv030aa1n03x5               g047(.a(new_n133), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n135), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n143), .b(new_n144), .c(new_n129), .d(new_n132), .o1(new_n145));
  aoi022aa1n02x7               g050(.a(new_n145), .b(new_n140), .c(new_n137), .d(new_n142), .o1(\s[12] ));
  oabi12aa1n03x5               g051(.a(new_n120), .b(new_n117), .c(new_n119), .out0(new_n147));
  nona23aa1n09x5               g052(.a(new_n139), .b(new_n134), .c(new_n133), .d(new_n138), .out0(new_n148));
  nor043aa1n09x5               g053(.a(new_n148), .b(new_n127), .c(new_n123), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n147), .c(new_n115), .d(new_n104), .o1(new_n150));
  nanb03aa1n09x5               g055(.a(new_n138), .b(new_n139), .c(new_n134), .out0(new_n151));
  oai112aa1n06x5               g056(.a(new_n143), .b(new_n126), .c(new_n125), .d(new_n97), .o1(new_n152));
  aoi012aa1d18x5               g057(.a(new_n138), .b(new_n133), .c(new_n139), .o1(new_n153));
  oai012aa1d24x5               g058(.a(new_n153), .b(new_n152), .c(new_n151), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nor022aa1n08x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand02aa1n03x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n150), .c(new_n155), .out0(\s[13] ));
  nand42aa1n03x5               g064(.a(new_n150), .b(new_n155), .o1(new_n160));
  nor042aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand02aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  aoi112aa1n02x5               g068(.a(new_n156), .b(new_n163), .c(new_n160), .d(new_n158), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n163), .b(new_n156), .c(new_n160), .d(new_n157), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(\s[14] ));
  nano23aa1n06x5               g071(.a(new_n156), .b(new_n161), .c(new_n162), .d(new_n157), .out0(new_n167));
  oaih12aa1n02x5               g072(.a(new_n162), .b(new_n161), .c(new_n156), .o1(new_n168));
  inv000aa1n02x5               g073(.a(new_n168), .o1(new_n169));
  nor022aa1n08x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  tech160nm_finand02aa1n03p5x5 g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n169), .c(new_n160), .d(new_n167), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n173), .b(new_n169), .c(new_n160), .d(new_n167), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  nor022aa1n08x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n04x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoib12aa1n02x5               g084(.a(new_n170), .b(new_n178), .c(new_n177), .out0(new_n180));
  oai012aa1n02x5               g085(.a(new_n174), .b(\b[14] ), .c(\a[15] ), .o1(new_n181));
  aboi22aa1n03x5               g086(.a(new_n179), .b(new_n181), .c(new_n174), .d(new_n180), .out0(\s[16] ));
  tech160nm_fiaoi012aa1n05x5   g087(.a(new_n147), .b(new_n115), .c(new_n104), .o1(new_n183));
  nona23aa1n02x4               g088(.a(new_n162), .b(new_n157), .c(new_n156), .d(new_n161), .out0(new_n184));
  nona23aa1d18x5               g089(.a(new_n178), .b(new_n171), .c(new_n170), .d(new_n177), .out0(new_n185));
  nona22aa1n09x5               g090(.a(new_n149), .b(new_n184), .c(new_n185), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n185), .o1(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n169), .c(new_n154), .d(new_n167), .o1(new_n188));
  aoi012aa1n02x7               g093(.a(new_n177), .b(new_n170), .c(new_n178), .o1(new_n189));
  oai112aa1n06x5               g094(.a(new_n188), .b(new_n189), .c(new_n183), .d(new_n186), .o1(new_n190));
  xorc02aa1n12x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  aoi012aa1n09x5               g096(.a(new_n186), .b(new_n116), .c(new_n121), .o1(new_n192));
  nano23aa1n02x4               g097(.a(new_n191), .b(new_n192), .c(new_n188), .d(new_n189), .out0(new_n193));
  aoi012aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(new_n191), .o1(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(\b[16] ), .b(new_n195), .out0(new_n196));
  nano22aa1n03x7               g101(.a(new_n138), .b(new_n134), .c(new_n139), .out0(new_n197));
  oai012aa1n02x5               g102(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .o1(new_n198));
  oab012aa1n02x4               g103(.a(new_n198), .b(new_n97), .c(new_n125), .out0(new_n199));
  inv000aa1n02x5               g104(.a(new_n153), .o1(new_n200));
  aoai13aa1n06x5               g105(.a(new_n167), .b(new_n200), .c(new_n199), .d(new_n197), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n189), .b(new_n185), .c(new_n201), .d(new_n168), .o1(new_n202));
  oaih12aa1n02x5               g107(.a(new_n191), .b(new_n202), .c(new_n192), .o1(new_n203));
  nor042aa1n06x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand02aa1n10x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n06x4               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(new_n196), .out0(\s[18] ));
  and002aa1n02x5               g112(.a(new_n191), .b(new_n206), .o(new_n208));
  oai012aa1n06x5               g113(.a(new_n208), .b(new_n202), .c(new_n192), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n12x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n12x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g121(.a(new_n214), .b(new_n210), .c(new_n190), .d(new_n208), .o1(new_n217));
  norp02aa1n24x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d28x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n06x4               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[18] ), .o1(new_n222));
  aboi22aa1n03x5               g127(.a(new_n218), .b(new_n219), .c(new_n221), .d(new_n222), .out0(new_n223));
  inv000aa1n04x5               g128(.a(new_n212), .o1(new_n224));
  inv000aa1n02x5               g129(.a(new_n214), .o1(new_n225));
  aoai13aa1n02x7               g130(.a(new_n224), .b(new_n225), .c(new_n209), .d(new_n211), .o1(new_n226));
  aoi022aa1n03x5               g131(.a(new_n226), .b(new_n220), .c(new_n217), .d(new_n223), .o1(\s[20] ));
  nano32aa1n03x7               g132(.a(new_n225), .b(new_n191), .c(new_n220), .d(new_n206), .out0(new_n228));
  oai012aa1n06x5               g133(.a(new_n228), .b(new_n202), .c(new_n192), .o1(new_n229));
  nanb03aa1n09x5               g134(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n230));
  nor042aa1n06x5               g135(.a(\b[16] ), .b(\a[17] ), .o1(new_n231));
  oai112aa1n06x5               g136(.a(new_n224), .b(new_n205), .c(new_n204), .d(new_n231), .o1(new_n232));
  tech160nm_fiaoi012aa1n05x5   g137(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n233));
  oai012aa1n18x5               g138(.a(new_n233), .b(new_n232), .c(new_n230), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n229), .c(new_n235), .out0(\s[21] ));
  aoai13aa1n03x5               g143(.a(new_n237), .b(new_n234), .c(new_n190), .d(new_n228), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[21] ), .b(\a[22] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nor042aa1n06x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n240), .b(new_n242), .out0(new_n243));
  inv000aa1n03x5               g148(.a(new_n242), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n236), .c(new_n229), .d(new_n235), .o1(new_n245));
  aoi022aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n239), .d(new_n243), .o1(\s[22] ));
  nor042aa1n06x5               g151(.a(new_n240), .b(new_n236), .o1(new_n247));
  and002aa1n02x5               g152(.a(new_n228), .b(new_n247), .o(new_n248));
  oaih12aa1n02x5               g153(.a(new_n248), .b(new_n202), .c(new_n192), .o1(new_n249));
  oao003aa1n12x5               g154(.a(\a[22] ), .b(\b[21] ), .c(new_n244), .carry(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n251), .b(new_n234), .c(new_n247), .o1(new_n252));
  inv030aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n190), .d(new_n248), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n251), .c(new_n234), .d(new_n247), .o1(new_n256));
  aobi12aa1n03x7               g161(.a(new_n255), .b(new_n256), .c(new_n249), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g162(.a(\a[24] ), .b(\b[23] ), .out0(new_n258));
  nor042aa1n06x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  norp02aa1n02x5               g164(.a(new_n258), .b(new_n259), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n259), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n254), .o1(new_n262));
  aoai13aa1n02x7               g167(.a(new_n261), .b(new_n262), .c(new_n249), .d(new_n252), .o1(new_n263));
  aoi022aa1n02x7               g168(.a(new_n263), .b(new_n258), .c(new_n255), .d(new_n260), .o1(\s[24] ));
  inv000aa1n02x5               g169(.a(new_n228), .o1(new_n265));
  and002aa1n06x5               g170(.a(new_n258), .b(new_n254), .o(new_n266));
  nano22aa1n02x5               g171(.a(new_n265), .b(new_n266), .c(new_n247), .out0(new_n267));
  oai012aa1n03x5               g172(.a(new_n267), .b(new_n202), .c(new_n192), .o1(new_n268));
  nano22aa1n02x5               g173(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n269));
  oai012aa1n02x5               g174(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .o1(new_n270));
  oab012aa1n04x5               g175(.a(new_n270), .b(new_n231), .c(new_n204), .out0(new_n271));
  inv020aa1n02x5               g176(.a(new_n233), .o1(new_n272));
  aoai13aa1n09x5               g177(.a(new_n247), .b(new_n272), .c(new_n271), .d(new_n269), .o1(new_n273));
  inv040aa1n02x5               g178(.a(new_n266), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[24] ), .b(\b[23] ), .c(new_n261), .carry(new_n275));
  aoai13aa1n12x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .d(new_n250), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n190), .d(new_n267), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n266), .b(new_n251), .c(new_n234), .d(new_n247), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n277), .o1(new_n280));
  and003aa1n02x5               g185(.a(new_n279), .b(new_n280), .c(new_n275), .o(new_n281));
  aobi12aa1n03x7               g186(.a(new_n278), .b(new_n281), .c(new_n268), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  nor042aa1n03x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n283), .b(new_n284), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n276), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n284), .o1(new_n287));
  aoai13aa1n02x7               g192(.a(new_n287), .b(new_n280), .c(new_n268), .d(new_n286), .o1(new_n288));
  aoi022aa1n03x5               g193(.a(new_n288), .b(new_n283), .c(new_n278), .d(new_n285), .o1(\s[26] ));
  and002aa1n18x5               g194(.a(new_n283), .b(new_n277), .o(new_n290));
  nano32aa1n03x7               g195(.a(new_n265), .b(new_n290), .c(new_n247), .d(new_n266), .out0(new_n291));
  oai012aa1n06x5               g196(.a(new_n291), .b(new_n202), .c(new_n192), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n290), .o1(new_n293));
  oao003aa1n06x5               g198(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n293), .c(new_n279), .d(new_n275), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n295), .c(new_n190), .d(new_n291), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n294), .o1(new_n298));
  aoi112aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n276), .d(new_n290), .o1(new_n299));
  aobi12aa1n02x7               g204(.a(new_n297), .b(new_n299), .c(new_n292), .out0(\s[27] ));
  xorc02aa1n02x5               g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  norp02aa1n02x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n301), .b(new_n302), .o1(new_n303));
  aoi012aa1n09x5               g208(.a(new_n298), .b(new_n276), .c(new_n290), .o1(new_n304));
  inv000aa1n03x5               g209(.a(new_n302), .o1(new_n305));
  inv000aa1n02x5               g210(.a(new_n296), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n306), .c(new_n304), .d(new_n292), .o1(new_n307));
  aoi022aa1n03x5               g212(.a(new_n307), .b(new_n301), .c(new_n297), .d(new_n303), .o1(\s[28] ));
  and002aa1n02x5               g213(.a(new_n301), .b(new_n296), .o(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n295), .c(new_n190), .d(new_n291), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n309), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n311), .c(new_n304), .d(new_n292), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .out0(new_n314));
  norb02aa1n02x5               g219(.a(new_n312), .b(new_n314), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n313), .b(new_n314), .c(new_n310), .d(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g222(.a(new_n306), .b(new_n301), .c(new_n314), .out0(new_n318));
  aoai13aa1n02x5               g223(.a(new_n318), .b(new_n295), .c(new_n190), .d(new_n291), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n318), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n320), .c(new_n304), .d(new_n292), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n321), .b(new_n323), .out0(new_n324));
  aoi022aa1n03x5               g229(.a(new_n322), .b(new_n323), .c(new_n319), .d(new_n324), .o1(\s[30] ));
  xorc02aa1n02x5               g230(.a(\a[31] ), .b(\b[30] ), .out0(new_n326));
  nano32aa1n06x5               g231(.a(new_n306), .b(new_n323), .c(new_n301), .d(new_n314), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n295), .c(new_n190), .d(new_n291), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n327), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n304), .d(new_n292), .o1(new_n331));
  and002aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .o(new_n332));
  oabi12aa1n02x5               g237(.a(new_n326), .b(\a[30] ), .c(\b[29] ), .out0(new_n333));
  oab012aa1n02x4               g238(.a(new_n333), .b(new_n321), .c(new_n332), .out0(new_n334));
  aoi022aa1n03x5               g239(.a(new_n331), .b(new_n326), .c(new_n328), .d(new_n334), .o1(\s[31] ));
  inv000aa1d42x5               g240(.a(\a[3] ), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n103), .b(\b[2] ), .c(new_n336), .out0(\s[3] ));
  orn002aa1n02x5               g242(.a(new_n99), .b(new_n103), .o(new_n338));
  xorc02aa1n02x5               g243(.a(\a[4] ), .b(\b[3] ), .out0(new_n339));
  aoib12aa1n02x5               g244(.a(new_n339), .b(new_n336), .c(\b[2] ), .out0(new_n340));
  aoi022aa1n02x5               g245(.a(new_n338), .b(new_n340), .c(new_n104), .d(new_n339), .o1(\s[4] ));
  and002aa1n02x5               g246(.a(\b[4] ), .b(\a[5] ), .o(new_n342));
  and002aa1n02x5               g247(.a(\b[3] ), .b(\a[4] ), .o(new_n343));
  nona32aa1n02x4               g248(.a(new_n104), .b(new_n113), .c(new_n343), .d(new_n342), .out0(new_n344));
  xnrc02aa1n02x5               g249(.a(\b[4] ), .b(\a[5] ), .out0(new_n345));
  nanb02aa1n02x5               g250(.a(new_n343), .b(new_n104), .out0(new_n346));
  aobi12aa1n02x5               g251(.a(new_n344), .b(new_n346), .c(new_n345), .out0(\s[5] ));
  inv000aa1d42x5               g252(.a(new_n113), .o1(new_n348));
  norb02aa1n02x5               g253(.a(new_n112), .b(new_n111), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n344), .c(new_n348), .out0(\s[6] ));
  oai022aa1n02x5               g255(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n351));
  aboi22aa1n03x5               g256(.a(new_n344), .b(new_n349), .c(new_n112), .d(new_n351), .out0(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n352), .b(new_n118), .c(new_n110), .out0(\s[7] ));
  nanb03aa1n02x5               g258(.a(new_n352), .b(new_n118), .c(new_n110), .out0(new_n354));
  oaoi03aa1n02x5               g259(.a(\a[7] ), .b(\b[6] ), .c(new_n352), .o1(new_n355));
  norb02aa1n02x5               g260(.a(new_n105), .b(new_n106), .out0(new_n356));
  aoib12aa1n02x5               g261(.a(new_n108), .b(new_n105), .c(new_n106), .out0(new_n357));
  aoi022aa1n02x5               g262(.a(new_n354), .b(new_n357), .c(new_n355), .d(new_n356), .o1(\s[8] ));
  xnbna2aa1n03x5               g263(.a(new_n124), .b(new_n116), .c(new_n121), .out0(\s[9] ));
endmodule


