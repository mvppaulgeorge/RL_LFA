// Benchmark "adder" written by ABC on Wed Jul 17 23:31:17 2024

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
    new_n118, new_n119, new_n120, new_n122, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n129, new_n130, new_n131, new_n133, new_n134,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n144, new_n145, new_n146, new_n147, new_n149, new_n150,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n163, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n179, new_n180, new_n181,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n210, new_n211, new_n212, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n226, new_n227, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n237, new_n238, new_n239,
    new_n241, new_n242, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n266, new_n267, new_n268, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n302, new_n304, new_n306, new_n307,
    new_n308;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[3] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[4] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[3] ), .o1(new_n99));
  aboi22aa1n03x5               g004(.a(\b[2] ), .b(new_n97), .c(new_n99), .d(new_n98), .out0(new_n100));
  tech160nm_fixnrc02aa1n02p5x5 g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  tech160nm_finand02aa1n05x5   g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1d24x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n12x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oaih12aa1n06x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  oai012aa1n09x5               g010(.a(new_n100), .b(new_n101), .c(new_n105), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  oai122aa1n06x5               g012(.a(new_n107), .b(\a[7] ), .c(\b[6] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n108));
  xnrc02aa1n12x5               g013(.a(\b[7] ), .b(\a[8] ), .out0(new_n109));
  aoi022aa1n12x5               g014(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n110));
  oai122aa1n06x5               g015(.a(new_n110), .b(\a[6] ), .c(\b[5] ), .d(new_n98), .e(new_n99), .o1(new_n111));
  nor043aa1d12x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .o1(new_n112));
  nor022aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  oab012aa1n02x4               g018(.a(new_n113), .b(\a[8] ), .c(\b[7] ), .out0(new_n114));
  nand02aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  oai022aa1d18x5               g020(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n116));
  nanp03aa1n02x5               g021(.a(new_n116), .b(new_n107), .c(new_n115), .o1(new_n117));
  aoi022aa1n09x5               g022(.a(new_n117), .b(new_n114), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  aoi012aa1d18x5               g023(.a(new_n118), .b(new_n112), .c(new_n106), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(\a[9] ), .b(\b[8] ), .c(new_n119), .o1(new_n120));
  xorb03aa1n02x5               g025(.a(new_n120), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d24x5               g026(.a(\b[9] ), .b(\a[10] ), .o1(new_n122));
  nand02aa1n06x5               g027(.a(\b[9] ), .b(\a[10] ), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[10] ), .b(\a[11] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  oaoi13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n120), .d(new_n122), .o1(new_n126));
  oai112aa1n04x5               g031(.a(new_n125), .b(new_n123), .c(new_n120), .d(new_n122), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(\s[11] ));
  nor042aa1d18x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(new_n129), .o1(new_n130));
  tech160nm_fixorc02aa1n03p5x5 g035(.a(\a[12] ), .b(\b[11] ), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n127), .c(new_n130), .out0(\s[12] ));
  nand42aa1n04x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nor042aa1n06x5               g039(.a(\b[8] ), .b(\a[9] ), .o1(new_n135));
  norb03aa1n03x5               g040(.a(new_n123), .b(new_n122), .c(new_n135), .out0(new_n136));
  nona23aa1n02x4               g041(.a(new_n131), .b(new_n136), .c(new_n124), .d(new_n134), .out0(new_n137));
  oab012aa1n03x5               g042(.a(new_n129), .b(\a[12] ), .c(\b[11] ), .out0(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  oai112aa1n03x5               g044(.a(new_n139), .b(new_n123), .c(new_n135), .d(new_n122), .o1(new_n140));
  aoi022aa1n02x7               g045(.a(new_n140), .b(new_n138), .c(\a[12] ), .d(\b[11] ), .o1(new_n141));
  oabi12aa1n09x5               g046(.a(new_n141), .b(new_n119), .c(new_n137), .out0(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g048(.a(\a[14] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\a[13] ), .o1(new_n145));
  inv000aa1d42x5               g050(.a(\b[12] ), .o1(new_n146));
  oaoi03aa1n02x5               g051(.a(new_n145), .b(new_n146), .c(new_n142), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[13] ), .c(new_n144), .out0(\s[14] ));
  xroi22aa1d04x5               g053(.a(new_n145), .b(\b[12] ), .c(new_n144), .d(\b[13] ), .out0(new_n149));
  norp02aa1n12x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  norp02aa1n24x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nanp02aa1n04x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  oa0012aa1n02x5               g057(.a(new_n152), .b(new_n151), .c(new_n150), .o(new_n153));
  nor002aa1n20x5               g058(.a(\b[14] ), .b(\a[15] ), .o1(new_n154));
  and002aa1n02x5               g059(.a(\b[14] ), .b(\a[15] ), .o(new_n155));
  norp02aa1n02x5               g060(.a(new_n155), .b(new_n154), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n153), .c(new_n142), .d(new_n149), .o1(new_n157));
  aoi112aa1n02x5               g062(.a(new_n156), .b(new_n153), .c(new_n142), .d(new_n149), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(\s[15] ));
  inv000aa1d42x5               g064(.a(new_n154), .o1(new_n160));
  xorc02aa1n02x5               g065(.a(\a[16] ), .b(\b[15] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n157), .c(new_n160), .out0(\s[16] ));
  nano22aa1n03x7               g067(.a(new_n124), .b(new_n131), .c(new_n133), .out0(new_n163));
  nona22aa1n02x4               g068(.a(new_n152), .b(new_n151), .c(new_n150), .out0(new_n164));
  oaih22aa1d12x5               g069(.a(new_n145), .b(new_n146), .c(\b[14] ), .d(\a[15] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\a[15] ), .o1(new_n166));
  inv000aa1d42x5               g071(.a(\b[14] ), .o1(new_n167));
  orn002aa1n24x5               g072(.a(\a[16] ), .b(\b[15] ), .o(new_n168));
  nand42aa1n03x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  oai112aa1n06x5               g074(.a(new_n168), .b(new_n169), .c(new_n167), .d(new_n166), .o1(new_n170));
  nor043aa1n06x5               g075(.a(new_n164), .b(new_n170), .c(new_n165), .o1(new_n171));
  nand23aa1n06x5               g076(.a(new_n163), .b(new_n136), .c(new_n171), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n168), .o1(new_n173));
  oaoi13aa1n03x5               g078(.a(new_n154), .b(new_n152), .c(new_n150), .d(new_n151), .o1(new_n174));
  aoi112aa1n03x5               g079(.a(new_n174), .b(new_n155), .c(\a[16] ), .d(\b[15] ), .o1(new_n175));
  aoi112aa1n09x5               g080(.a(new_n173), .b(new_n175), .c(new_n141), .d(new_n171), .o1(new_n176));
  oai012aa1d24x5               g081(.a(new_n176), .b(new_n172), .c(new_n119), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g083(.a(\a[18] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\a[17] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\b[16] ), .o1(new_n181));
  oaoi03aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n177), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(new_n179), .out0(\s[18] ));
  xroi22aa1d06x4               g088(.a(new_n180), .b(\b[16] ), .c(new_n179), .d(\b[17] ), .out0(new_n184));
  oai022aa1d24x5               g089(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n185));
  oaib12aa1n18x5               g090(.a(new_n185), .b(new_n179), .c(\b[17] ), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  nor042aa1n04x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nand02aa1d04x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n187), .c(new_n177), .d(new_n184), .o1(new_n191));
  aoi112aa1n02x5               g096(.a(new_n190), .b(new_n187), .c(new_n177), .d(new_n184), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g098(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  orn002aa1n02x5               g099(.a(\a[19] ), .b(\b[18] ), .o(new_n195));
  nor042aa1n03x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nand02aa1n04x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n191), .c(new_n195), .out0(\s[20] ));
  nano23aa1n06x5               g104(.a(new_n188), .b(new_n196), .c(new_n197), .d(new_n189), .out0(new_n200));
  nand22aa1n04x5               g105(.a(new_n184), .b(new_n200), .o1(new_n201));
  inv040aa1n08x5               g106(.a(new_n201), .o1(new_n202));
  nona23aa1n09x5               g107(.a(new_n197), .b(new_n189), .c(new_n188), .d(new_n196), .out0(new_n203));
  aoi012aa1n02x7               g108(.a(new_n196), .b(new_n188), .c(new_n197), .o1(new_n204));
  oai012aa1n06x5               g109(.a(new_n204), .b(new_n203), .c(new_n186), .o1(new_n205));
  tech160nm_fixorc02aa1n03p5x5 g110(.a(\a[21] ), .b(\b[20] ), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n205), .c(new_n177), .d(new_n202), .o1(new_n207));
  aoi112aa1n02x5               g112(.a(new_n206), .b(new_n205), .c(new_n177), .d(new_n202), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(\s[21] ));
  inv000aa1d42x5               g114(.a(\a[21] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(\b[20] ), .b(new_n210), .out0(new_n211));
  xorc02aa1n02x5               g116(.a(\a[22] ), .b(\b[21] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n211), .out0(\s[22] ));
  nanp02aa1n02x5               g118(.a(new_n212), .b(new_n206), .o1(new_n214));
  nano22aa1n02x4               g119(.a(new_n214), .b(new_n184), .c(new_n200), .out0(new_n215));
  inv020aa1n03x5               g120(.a(new_n204), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\a[22] ), .o1(new_n217));
  xroi22aa1d04x5               g122(.a(new_n210), .b(\b[20] ), .c(new_n217), .d(\b[21] ), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n216), .c(new_n200), .d(new_n187), .o1(new_n219));
  oao003aa1n12x5               g124(.a(\a[22] ), .b(\b[21] ), .c(new_n211), .carry(new_n220));
  nanp02aa1n02x5               g125(.a(new_n219), .b(new_n220), .o1(new_n221));
  xorc02aa1n12x5               g126(.a(\a[23] ), .b(\b[22] ), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n221), .c(new_n177), .d(new_n215), .o1(new_n223));
  aoi112aa1n02x5               g128(.a(new_n222), .b(new_n221), .c(new_n177), .d(new_n215), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n223), .b(new_n224), .out0(\s[23] ));
  orn002aa1n02x5               g130(.a(\a[23] ), .b(\b[22] ), .o(new_n226));
  xorc02aa1n12x5               g131(.a(\a[24] ), .b(\b[23] ), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n223), .c(new_n226), .out0(\s[24] ));
  nand02aa1d06x5               g133(.a(new_n227), .b(new_n222), .o1(new_n229));
  nor043aa1n02x5               g134(.a(new_n201), .b(new_n214), .c(new_n229), .o1(new_n230));
  oao003aa1n02x5               g135(.a(\a[24] ), .b(\b[23] ), .c(new_n226), .carry(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n229), .c(new_n219), .d(new_n220), .o1(new_n232));
  aoi012aa1n03x5               g137(.a(new_n232), .b(new_n177), .c(new_n230), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[24] ), .b(\a[25] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnrc02aa1n03x5               g140(.a(new_n233), .b(new_n235), .out0(\s[25] ));
  orn002aa1n02x5               g141(.a(\a[25] ), .b(\b[24] ), .o(new_n237));
  aoai13aa1n06x5               g142(.a(new_n235), .b(new_n232), .c(new_n177), .d(new_n230), .o1(new_n238));
  xnrc02aa1n02x5               g143(.a(\b[25] ), .b(\a[26] ), .out0(new_n239));
  xobna2aa1n03x5               g144(.a(new_n239), .b(new_n238), .c(new_n237), .out0(\s[26] ));
  nor042aa1n04x5               g145(.a(new_n239), .b(new_n234), .o1(new_n241));
  nano23aa1d12x5               g146(.a(new_n201), .b(new_n229), .c(new_n241), .d(new_n218), .out0(new_n242));
  nand02aa1d10x5               g147(.a(new_n177), .b(new_n242), .o1(new_n243));
  nand22aa1n03x5               g148(.a(new_n232), .b(new_n241), .o1(new_n244));
  oao003aa1n02x5               g149(.a(\a[26] ), .b(\b[25] ), .c(new_n237), .carry(new_n245));
  nor042aa1n03x5               g150(.a(\b[26] ), .b(\a[27] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[26] ), .b(\a[27] ), .o1(new_n247));
  norb02aa1n12x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  aoi013aa1n06x5               g154(.a(new_n249), .b(new_n243), .c(new_n244), .d(new_n245), .o1(new_n250));
  inv020aa1n03x5               g155(.a(new_n241), .o1(new_n251));
  nona32aa1n03x5               g156(.a(new_n202), .b(new_n251), .c(new_n229), .d(new_n214), .out0(new_n252));
  oaoi13aa1n04x5               g157(.a(new_n252), .b(new_n176), .c(new_n119), .d(new_n172), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n220), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n229), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n205), .d(new_n218), .o1(new_n256));
  aoai13aa1n06x5               g161(.a(new_n245), .b(new_n251), .c(new_n256), .d(new_n231), .o1(new_n257));
  norp03aa1n02x5               g162(.a(new_n257), .b(new_n253), .c(new_n248), .o1(new_n258));
  norp02aa1n02x5               g163(.a(new_n250), .b(new_n258), .o1(\s[27] ));
  inv020aa1n02x5               g164(.a(new_n246), .o1(new_n260));
  xnrc02aa1n02x5               g165(.a(\b[27] ), .b(\a[28] ), .out0(new_n261));
  nano22aa1n03x7               g166(.a(new_n250), .b(new_n260), .c(new_n261), .out0(new_n262));
  oaih12aa1n02x5               g167(.a(new_n248), .b(new_n257), .c(new_n253), .o1(new_n263));
  tech160nm_fiaoi012aa1n02p5x5 g168(.a(new_n261), .b(new_n263), .c(new_n260), .o1(new_n264));
  nor002aa1n02x5               g169(.a(new_n264), .b(new_n262), .o1(\s[28] ));
  nano22aa1n06x5               g170(.a(new_n261), .b(new_n260), .c(new_n247), .out0(new_n266));
  oaih12aa1n02x5               g171(.a(new_n266), .b(new_n257), .c(new_n253), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[28] ), .b(\b[27] ), .c(new_n260), .carry(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[28] ), .b(\a[29] ), .out0(new_n269));
  tech160nm_fiaoi012aa1n02p5x5 g174(.a(new_n269), .b(new_n267), .c(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n266), .o1(new_n271));
  aoi013aa1n03x5               g176(.a(new_n271), .b(new_n243), .c(new_n244), .d(new_n245), .o1(new_n272));
  nano22aa1n03x5               g177(.a(new_n272), .b(new_n268), .c(new_n269), .out0(new_n273));
  nor002aa1n02x5               g178(.a(new_n270), .b(new_n273), .o1(\s[29] ));
  xorb03aa1n02x5               g179(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g180(.a(new_n269), .b(new_n261), .c(new_n247), .d(new_n260), .out0(new_n276));
  oaih12aa1n02x5               g181(.a(new_n276), .b(new_n257), .c(new_n253), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[29] ), .b(\b[28] ), .c(new_n268), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[29] ), .b(\a[30] ), .out0(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n278), .o1(new_n280));
  inv000aa1n02x5               g185(.a(new_n276), .o1(new_n281));
  aoi013aa1n03x5               g186(.a(new_n281), .b(new_n243), .c(new_n244), .d(new_n245), .o1(new_n282));
  nano22aa1n03x5               g187(.a(new_n282), .b(new_n278), .c(new_n279), .out0(new_n283));
  norp02aa1n03x5               g188(.a(new_n280), .b(new_n283), .o1(\s[30] ));
  norb03aa1n02x5               g189(.a(new_n266), .b(new_n279), .c(new_n269), .out0(new_n285));
  inv000aa1n02x5               g190(.a(new_n285), .o1(new_n286));
  aoi013aa1n03x5               g191(.a(new_n286), .b(new_n243), .c(new_n244), .d(new_n245), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[30] ), .b(\b[29] ), .c(new_n278), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[30] ), .b(\a[31] ), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n287), .b(new_n288), .c(new_n289), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n285), .b(new_n257), .c(new_n253), .o1(new_n291));
  aoi012aa1n03x5               g196(.a(new_n289), .b(new_n291), .c(new_n288), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n290), .o1(\s[31] ));
  xorb03aa1n02x5               g198(.a(new_n105), .b(\b[2] ), .c(new_n97), .out0(\s[3] ));
  xnrc02aa1n02x5               g199(.a(\b[3] ), .b(\a[4] ), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .carry(new_n296));
  mtn022aa1n02x5               g201(.a(new_n296), .b(new_n106), .sa(new_n295), .o1(\s[4] ));
  xorc02aa1n02x5               g202(.a(\a[5] ), .b(\b[4] ), .out0(new_n298));
  oaoi13aa1n02x5               g203(.a(new_n298), .b(new_n106), .c(new_n98), .d(new_n99), .o1(new_n299));
  oai112aa1n02x5               g204(.a(new_n106), .b(new_n298), .c(new_n99), .d(new_n98), .o1(new_n300));
  norb02aa1n02x5               g205(.a(new_n300), .b(new_n299), .out0(\s[5] ));
  oa0012aa1n02x5               g206(.a(new_n300), .b(\b[4] ), .c(\a[5] ), .o(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g208(.a(\a[6] ), .b(\b[5] ), .c(new_n302), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  inv000aa1d42x5               g210(.a(new_n109), .o1(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n113), .c(new_n304), .d(new_n115), .o1(new_n307));
  aoi112aa1n02x5               g212(.a(new_n113), .b(new_n306), .c(new_n304), .d(new_n115), .o1(new_n308));
  norb02aa1n02x5               g213(.a(new_n307), .b(new_n308), .out0(\s[8] ));
  xnrb03aa1n02x5               g214(.a(new_n119), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


