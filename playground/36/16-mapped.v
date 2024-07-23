// Benchmark "adder" written by ABC on Thu Jul 18 06:31:44 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n317, new_n319, new_n320, new_n323, new_n325, new_n327, new_n328,
    new_n329, new_n331;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n09x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1d18x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  norp02aa1n12x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1d18x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fioai012aa1n03p5x5 g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1d24x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand22aa1n09x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand22aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1d18x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_finand02aa1n05x5   g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norp02aa1n24x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand42aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nona23aa1n06x5               g023(.a(new_n115), .b(new_n118), .c(new_n117), .d(new_n116), .out0(new_n119));
  nor042aa1n06x5               g024(.a(new_n119), .b(new_n114), .o1(new_n120));
  tech160nm_fioai012aa1n03p5x5 g025(.a(new_n115), .b(new_n117), .c(new_n116), .o1(new_n121));
  oaih12aa1n02x5               g026(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n122));
  oai012aa1n09x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  nand22aa1n03x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n120), .o1(new_n126));
  nor002aa1n20x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1d08x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  nona23aa1n02x4               g034(.a(new_n126), .b(new_n128), .c(new_n127), .d(new_n97), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n98), .d(new_n126), .o1(\s[10] ));
  nano23aa1n06x5               g036(.a(new_n97), .b(new_n127), .c(new_n128), .d(new_n124), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n123), .c(new_n109), .d(new_n120), .o1(new_n133));
  oai012aa1d24x5               g038(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n134));
  nor022aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n137), .b(new_n133), .c(new_n134), .out0(\s[11] ));
  inv030aa1n02x5               g043(.a(new_n135), .o1(new_n139));
  nanp02aa1n02x5               g044(.a(new_n133), .b(new_n134), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n136), .o1(new_n141));
  nona22aa1n02x4               g046(.a(new_n140), .b(new_n135), .c(new_n141), .out0(new_n142));
  nor002aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xobna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n139), .out0(\s[12] ));
  nano23aa1n03x7               g051(.a(new_n145), .b(new_n137), .c(new_n129), .d(new_n125), .out0(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n123), .c(new_n109), .d(new_n120), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n143), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n141), .c(new_n134), .d(new_n139), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n150), .b(new_n144), .o1(new_n151));
  nor042aa1d18x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand02aa1n08x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(new_n152), .o1(new_n156));
  aob012aa1n02x5               g061(.a(new_n154), .b(new_n148), .c(new_n151), .out0(new_n157));
  nor042aa1n03x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nand42aa1n16x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  oai022aa1d18x5               g065(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n161));
  nanb03aa1n02x5               g066(.a(new_n161), .b(new_n157), .c(new_n159), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n160), .c(new_n156), .d(new_n157), .o1(\s[14] ));
  nano23aa1n09x5               g068(.a(new_n152), .b(new_n158), .c(new_n159), .d(new_n153), .out0(new_n164));
  inv040aa1n03x5               g069(.a(new_n164), .o1(new_n165));
  oai012aa1n02x5               g070(.a(new_n159), .b(new_n158), .c(new_n152), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n165), .c(new_n148), .d(new_n151), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  aoi012aa1n02x5               g075(.a(new_n169), .b(new_n167), .c(new_n170), .o1(new_n171));
  nor002aa1n03x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand02aa1n20x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n02x7               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  norb02aa1n12x5               g079(.a(new_n170), .b(new_n169), .out0(new_n175));
  norb03aa1n02x5               g080(.a(new_n173), .b(new_n169), .c(new_n172), .out0(new_n176));
  aob012aa1n03x5               g081(.a(new_n176), .b(new_n167), .c(new_n175), .out0(new_n177));
  oai012aa1n02x5               g082(.a(new_n177), .b(new_n171), .c(new_n174), .o1(\s[16] ));
  nano23aa1n03x5               g083(.a(new_n135), .b(new_n143), .c(new_n144), .d(new_n136), .out0(new_n179));
  nano23aa1n02x4               g084(.a(new_n169), .b(new_n172), .c(new_n173), .d(new_n170), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n164), .o1(new_n181));
  nano22aa1n03x7               g086(.a(new_n181), .b(new_n132), .c(new_n179), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n123), .c(new_n109), .d(new_n120), .o1(new_n183));
  aoi022aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n184));
  nano32aa1n03x7               g089(.a(new_n165), .b(new_n184), .c(new_n175), .d(new_n174), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n170), .b(new_n169), .c(new_n161), .d(new_n159), .o1(new_n186));
  oai012aa1n03x5               g091(.a(new_n186), .b(\b[15] ), .c(\a[16] ), .o1(new_n187));
  aoi022aa1n12x5               g092(.a(new_n185), .b(new_n150), .c(new_n173), .d(new_n187), .o1(new_n188));
  xorc02aa1n02x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n183), .c(new_n188), .out0(\s[17] ));
  nor002aa1n12x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nand42aa1n10x5               g097(.a(new_n183), .b(new_n188), .o1(new_n193));
  nanp02aa1n06x5               g098(.a(new_n193), .b(new_n189), .o1(new_n194));
  xorc02aa1n02x5               g099(.a(\a[18] ), .b(\b[17] ), .out0(new_n195));
  nand02aa1n08x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  oai022aa1d18x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  nanb03aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n196), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n195), .c(new_n192), .d(new_n194), .o1(\s[18] ));
  nand02aa1n06x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  nor002aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nano23aa1n06x5               g106(.a(new_n191), .b(new_n201), .c(new_n196), .d(new_n200), .out0(new_n202));
  oaoi03aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n203));
  xorc02aa1n12x5               g108(.a(\a[19] ), .b(\b[18] ), .out0(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n203), .c(new_n193), .d(new_n202), .o1(new_n205));
  aoi112aa1n02x5               g110(.a(new_n204), .b(new_n203), .c(new_n193), .d(new_n202), .o1(new_n206));
  norb02aa1n03x4               g111(.a(new_n205), .b(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g113(.a(\a[19] ), .o1(new_n209));
  inv000aa1d42x5               g114(.a(\b[18] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n210), .b(new_n209), .o1(new_n211));
  tech160nm_fixorc02aa1n03p5x5 g116(.a(\a[20] ), .b(\b[19] ), .out0(new_n212));
  nand02aa1n08x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  oai022aa1n02x7               g119(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n215));
  nona22aa1n03x5               g120(.a(new_n205), .b(new_n214), .c(new_n215), .out0(new_n216));
  aoai13aa1n03x5               g121(.a(new_n216), .b(new_n212), .c(new_n211), .d(new_n205), .o1(\s[20] ));
  nanp03aa1d12x5               g122(.a(new_n202), .b(new_n204), .c(new_n212), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  oai112aa1n03x5               g124(.a(new_n197), .b(new_n196), .c(new_n210), .d(new_n209), .o1(new_n220));
  aoib12aa1n02x5               g125(.a(new_n214), .b(new_n220), .c(new_n215), .out0(new_n221));
  nor042aa1n09x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n221), .c(new_n193), .d(new_n219), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(new_n224), .b(new_n221), .c(new_n193), .d(new_n219), .o1(new_n226));
  norb02aa1n03x4               g131(.a(new_n225), .b(new_n226), .out0(\s[21] ));
  inv000aa1d42x5               g132(.a(new_n222), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  and002aa1n02x5               g135(.a(\b[21] ), .b(\a[22] ), .o(new_n231));
  oai022aa1n02x5               g136(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n232));
  nona22aa1n02x5               g137(.a(new_n225), .b(new_n231), .c(new_n232), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n230), .c(new_n228), .d(new_n225), .o1(\s[22] ));
  nano22aa1n06x5               g139(.a(new_n229), .b(new_n228), .c(new_n223), .out0(new_n235));
  norb02aa1n03x5               g140(.a(new_n235), .b(new_n218), .out0(new_n236));
  nanb02aa1n02x5               g141(.a(new_n215), .b(new_n220), .out0(new_n237));
  nano32aa1n03x7               g142(.a(new_n229), .b(new_n228), .c(new_n223), .d(new_n213), .out0(new_n238));
  nanp02aa1n02x5               g143(.a(new_n237), .b(new_n238), .o1(new_n239));
  oaib12aa1n02x5               g144(.a(new_n239), .b(new_n231), .c(new_n232), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n193), .d(new_n236), .o1(new_n242));
  oaoi03aa1n03x5               g147(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .o1(new_n243));
  nona22aa1n02x4               g148(.a(new_n239), .b(new_n243), .c(new_n241), .out0(new_n244));
  aoi012aa1n02x5               g149(.a(new_n244), .b(new_n193), .c(new_n236), .o1(new_n245));
  norb02aa1n03x4               g150(.a(new_n242), .b(new_n245), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  and002aa1n02x5               g154(.a(\b[23] ), .b(\a[24] ), .o(new_n250));
  oai022aa1n02x5               g155(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n251));
  nona22aa1n02x5               g156(.a(new_n242), .b(new_n250), .c(new_n251), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n249), .c(new_n248), .d(new_n242), .o1(\s[24] ));
  inv000aa1d42x5               g158(.a(\a[23] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(\a[24] ), .o1(new_n255));
  xroi22aa1d06x4               g160(.a(new_n254), .b(\b[22] ), .c(new_n255), .d(\b[23] ), .out0(new_n256));
  nano22aa1n12x5               g161(.a(new_n218), .b(new_n256), .c(new_n235), .out0(new_n257));
  oaib12aa1n02x5               g162(.a(new_n251), .b(new_n255), .c(\b[23] ), .out0(new_n258));
  aoai13aa1n04x5               g163(.a(new_n256), .b(new_n243), .c(new_n237), .d(new_n238), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n259), .b(new_n258), .o1(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[24] ), .b(\a[25] ), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n260), .c(new_n193), .d(new_n257), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n262), .b(new_n260), .c(new_n193), .d(new_n257), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n263), .b(new_n264), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  xorc02aa1n03x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  and002aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .o(new_n269));
  oai022aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n270));
  nona22aa1n03x5               g175(.a(new_n263), .b(new_n269), .c(new_n270), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n268), .c(new_n267), .d(new_n263), .o1(\s[26] ));
  norb02aa1n09x5               g177(.a(new_n268), .b(new_n261), .out0(new_n273));
  nano32aa1d12x5               g178(.a(new_n218), .b(new_n273), .c(new_n235), .d(new_n256), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n273), .o1(new_n275));
  aob012aa1n02x5               g180(.a(new_n270), .b(\b[25] ), .c(\a[26] ), .out0(new_n276));
  aoai13aa1n09x5               g181(.a(new_n276), .b(new_n275), .c(new_n259), .d(new_n258), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n277), .c(new_n193), .d(new_n274), .o1(new_n279));
  aoi112aa1n02x5               g184(.a(new_n277), .b(new_n278), .c(new_n193), .d(new_n274), .o1(new_n280));
  norb02aa1n03x4               g185(.a(new_n279), .b(new_n280), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .out0(new_n284));
  oai022aa1d24x5               g189(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(\a[28] ), .c(\b[27] ), .o1(new_n286));
  tech160nm_finand02aa1n05x5   g191(.a(new_n279), .b(new_n286), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n284), .c(new_n283), .d(new_n279), .o1(\s[28] ));
  xorc02aa1n02x5               g193(.a(\a[29] ), .b(\b[28] ), .out0(new_n289));
  inv020aa1n04x5               g194(.a(new_n274), .o1(new_n290));
  tech160nm_fiaoi012aa1n03p5x5 g195(.a(new_n290), .b(new_n183), .c(new_n188), .o1(new_n291));
  and002aa1n02x5               g196(.a(new_n284), .b(new_n278), .o(new_n292));
  inv000aa1d42x5               g197(.a(\b[27] ), .o1(new_n293));
  oaib12aa1n09x5               g198(.a(new_n285), .b(new_n293), .c(\a[28] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  oaoi13aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n291), .d(new_n277), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n292), .b(new_n277), .c(new_n193), .d(new_n274), .o1(new_n297));
  nand03aa1n02x5               g202(.a(new_n297), .b(new_n289), .c(new_n294), .o1(new_n298));
  oai012aa1n03x5               g203(.a(new_n298), .b(new_n296), .c(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g205(.a(new_n278), .b(new_n289), .c(new_n284), .o(new_n301));
  tech160nm_fioaoi03aa1n03p5x5 g206(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .o1(new_n302));
  oaoi13aa1n03x5               g207(.a(new_n302), .b(new_n301), .c(new_n291), .d(new_n277), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n301), .b(new_n277), .c(new_n193), .d(new_n274), .o1(new_n305));
  norb02aa1n02x5               g210(.a(new_n304), .b(new_n302), .out0(new_n306));
  nand42aa1n02x5               g211(.a(new_n305), .b(new_n306), .o1(new_n307));
  tech160nm_fioai012aa1n02p5x5 g212(.a(new_n307), .b(new_n303), .c(new_n304), .o1(\s[30] ));
  and003aa1n02x5               g213(.a(new_n292), .b(new_n289), .c(new_n304), .o(new_n309));
  aoai13aa1n06x5               g214(.a(new_n309), .b(new_n277), .c(new_n193), .d(new_n274), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n311));
  xorc02aa1n02x5               g216(.a(\a[31] ), .b(\b[30] ), .out0(new_n312));
  oai012aa1n02x5               g217(.a(new_n312), .b(\b[29] ), .c(\a[30] ), .o1(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(new_n302), .c(new_n304), .o1(new_n314));
  nanp02aa1n06x5               g219(.a(new_n310), .b(new_n314), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n312), .c(new_n310), .d(new_n311), .o1(\s[31] ));
  inv000aa1d42x5               g221(.a(new_n105), .o1(new_n317));
  xnbna2aa1n03x5               g222(.a(new_n102), .b(new_n106), .c(new_n317), .out0(\s[3] ));
  norb02aa1n02x5               g223(.a(new_n104), .b(new_n103), .out0(new_n319));
  nanb03aa1n02x5               g224(.a(new_n102), .b(new_n106), .c(new_n317), .out0(new_n320));
  xnbna2aa1n03x5               g225(.a(new_n319), .b(new_n320), .c(new_n317), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g227(.a(new_n117), .b(new_n109), .c(new_n118), .o1(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g229(.a(new_n121), .b(new_n119), .c(new_n109), .out0(new_n325));
  xorb03aa1n02x5               g230(.a(new_n325), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  norb02aa1n02x5               g231(.a(new_n111), .b(new_n110), .out0(new_n327));
  inv000aa1d42x5               g232(.a(new_n112), .o1(new_n328));
  nanp03aa1n02x5               g233(.a(new_n325), .b(new_n328), .c(new_n113), .o1(new_n329));
  xnbna2aa1n03x5               g234(.a(new_n327), .b(new_n329), .c(new_n328), .out0(\s[8] ));
  aoi112aa1n02x5               g235(.a(new_n125), .b(new_n123), .c(new_n109), .d(new_n120), .o1(new_n331));
  norb02aa1n02x5               g236(.a(new_n126), .b(new_n331), .out0(\s[9] ));
endmodule


