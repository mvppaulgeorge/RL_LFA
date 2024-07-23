// Benchmark "adder" written by ABC on Wed Jul 17 20:18:40 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n328, new_n331, new_n333, new_n334,
    new_n335, new_n336, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  inv040aa1d32x5               g003(.a(\a[3] ), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\b[2] ), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand02aa1n02x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  nor042aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nand42aa1n08x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand42aa1n16x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  aoi012aa1n12x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\a[4] ), .o1(new_n108));
  aboi22aa1n03x5               g013(.a(\b[3] ), .b(new_n108), .c(new_n99), .d(new_n100), .out0(new_n109));
  oaoi13aa1n09x5               g014(.a(new_n98), .b(new_n109), .c(new_n107), .d(new_n103), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n06x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand22aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1d18x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  norp03aa1d12x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  and002aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o(new_n119));
  inv000aa1d42x5               g024(.a(\a[5] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[6] ), .o1(new_n121));
  inv020aa1d32x5               g026(.a(\b[4] ), .o1(new_n122));
  aboi22aa1n06x5               g027(.a(\b[5] ), .b(new_n121), .c(new_n120), .d(new_n122), .out0(new_n123));
  oaih12aa1n02x5               g028(.a(new_n112), .b(new_n114), .c(new_n111), .o1(new_n124));
  oai013aa1n06x5               g029(.a(new_n124), .b(new_n115), .c(new_n119), .d(new_n123), .o1(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n127));
  norp02aa1n12x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n06x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g036(.a(new_n128), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n129), .o1(new_n133));
  nor042aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1n02x5               g041(.a(new_n136), .o1(new_n137));
  aoi113aa1n02x5               g042(.a(new_n137), .b(new_n133), .c(new_n127), .d(new_n132), .e(new_n97), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n127), .b(new_n97), .o1(new_n139));
  oaoi03aa1n09x5               g044(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n140));
  aoi112aa1n02x5               g045(.a(new_n140), .b(new_n136), .c(new_n139), .d(new_n130), .o1(new_n141));
  norp02aa1n02x5               g046(.a(new_n141), .b(new_n138), .o1(\s[11] ));
  nor042aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  norp03aa1n02x5               g050(.a(new_n138), .b(new_n145), .c(new_n134), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n134), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n136), .b(new_n140), .c(new_n139), .d(new_n130), .o1(new_n148));
  aobi12aa1n06x5               g053(.a(new_n145), .b(new_n148), .c(new_n147), .out0(new_n149));
  norp02aa1n02x5               g054(.a(new_n149), .b(new_n146), .o1(\s[12] ));
  nano23aa1d15x5               g055(.a(new_n134), .b(new_n143), .c(new_n144), .d(new_n135), .out0(new_n151));
  and003aa1n02x5               g056(.a(new_n151), .b(new_n130), .c(new_n126), .o(new_n152));
  aoai13aa1n03x5               g057(.a(new_n152), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n153));
  aoi112aa1n02x5               g058(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n154));
  aoi112aa1n03x5               g059(.a(new_n154), .b(new_n143), .c(new_n151), .d(new_n140), .o1(new_n155));
  nor002aa1n16x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n06x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n153), .c(new_n155), .out0(\s[13] ));
  inv000aa1d42x5               g064(.a(new_n156), .o1(new_n160));
  oai012aa1n02x5               g065(.a(new_n109), .b(new_n107), .c(new_n103), .o1(new_n161));
  oaib12aa1n06x5               g066(.a(new_n161), .b(new_n108), .c(\b[3] ), .out0(new_n162));
  nano23aa1n03x7               g067(.a(new_n114), .b(new_n111), .c(new_n112), .d(new_n113), .out0(new_n163));
  nona22aa1n02x4               g068(.a(new_n163), .b(new_n116), .c(new_n117), .out0(new_n164));
  norp02aa1n02x5               g069(.a(new_n123), .b(new_n119), .o1(new_n165));
  aobi12aa1n06x5               g070(.a(new_n124), .b(new_n163), .c(new_n165), .out0(new_n166));
  oai012aa1n06x5               g071(.a(new_n166), .b(new_n162), .c(new_n164), .o1(new_n167));
  nand22aa1n03x5               g072(.a(new_n151), .b(new_n140), .o1(new_n168));
  nona22aa1n03x5               g073(.a(new_n168), .b(new_n154), .c(new_n143), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n158), .b(new_n169), .c(new_n167), .d(new_n152), .o1(new_n170));
  nor002aa1n03x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand02aa1n06x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  norb02aa1n03x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n160), .out0(\s[14] ));
  nona23aa1n02x4               g079(.a(new_n172), .b(new_n157), .c(new_n156), .d(new_n171), .out0(new_n175));
  tech160nm_fioai012aa1n04x5   g080(.a(new_n172), .b(new_n171), .c(new_n156), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n176), .b(new_n175), .c(new_n153), .d(new_n155), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand42aa1n06x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nor042aa1n03x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand02aa1n06x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n179), .c(new_n177), .d(new_n180), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n183), .b(new_n179), .c(new_n177), .d(new_n180), .o1(new_n185));
  norb02aa1n02x7               g090(.a(new_n185), .b(new_n184), .out0(\s[16] ));
  nano23aa1n06x5               g091(.a(new_n179), .b(new_n181), .c(new_n182), .d(new_n180), .out0(new_n187));
  nand23aa1n06x5               g092(.a(new_n187), .b(new_n158), .c(new_n173), .o1(new_n188));
  nano32aa1n03x7               g093(.a(new_n188), .b(new_n151), .c(new_n130), .d(new_n126), .out0(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n190));
  inv020aa1n03x5               g095(.a(new_n188), .o1(new_n191));
  nona23aa1n03x5               g096(.a(new_n182), .b(new_n180), .c(new_n179), .d(new_n181), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(new_n179), .b(new_n182), .o1(new_n193));
  oai122aa1n06x5               g098(.a(new_n193), .b(new_n192), .c(new_n176), .d(\b[15] ), .e(\a[16] ), .o1(new_n194));
  aoi012aa1n12x5               g099(.a(new_n194), .b(new_n169), .c(new_n191), .o1(new_n195));
  xorc02aa1n12x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n196), .b(new_n190), .c(new_n195), .out0(\s[17] ));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[16] ), .o1(new_n199));
  nanp02aa1n09x5               g104(.a(new_n190), .b(new_n195), .o1(new_n200));
  tech160nm_fioaoi03aa1n03p5x5 g105(.a(new_n198), .b(new_n199), .c(new_n200), .o1(new_n201));
  nor002aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nanb02aa1n06x5               g108(.a(new_n202), .b(new_n203), .out0(new_n204));
  xorc02aa1n02x5               g109(.a(new_n201), .b(new_n204), .out0(\s[18] ));
  norb02aa1n03x5               g110(.a(new_n196), .b(new_n204), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n203), .b(new_n202), .c(new_n198), .d(new_n199), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n207), .c(new_n190), .d(new_n195), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1d18x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand22aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\b[19] ), .o1(new_n214));
  nanb02aa1n09x5               g119(.a(\a[20] ), .b(new_n214), .out0(new_n215));
  nand02aa1n04x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  aoi122aa1n03x5               g121(.a(new_n212), .b(new_n216), .c(new_n215), .d(new_n209), .e(new_n213), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n212), .o1(new_n218));
  nanb02aa1n12x5               g123(.a(new_n212), .b(new_n213), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  nand02aa1n02x5               g125(.a(new_n209), .b(new_n220), .o1(new_n221));
  nand42aa1n03x5               g126(.a(new_n215), .b(new_n216), .o1(new_n222));
  tech160nm_fiaoi012aa1n02p5x5 g127(.a(new_n222), .b(new_n221), .c(new_n218), .o1(new_n223));
  norp02aa1n03x5               g128(.a(new_n223), .b(new_n217), .o1(\s[20] ));
  nona23aa1n02x4               g129(.a(new_n220), .b(new_n196), .c(new_n204), .d(new_n222), .out0(new_n225));
  nand42aa1n03x5               g130(.a(new_n212), .b(new_n216), .o1(new_n226));
  nor003aa1n04x5               g131(.a(new_n208), .b(new_n219), .c(new_n222), .o1(new_n227));
  nano22aa1n03x7               g132(.a(new_n227), .b(new_n215), .c(new_n226), .out0(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n225), .c(new_n190), .d(new_n195), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[20] ), .b(\a[21] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi112aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n229), .d(new_n233), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n235), .b(new_n231), .c(new_n229), .d(new_n233), .o1(new_n237));
  norb02aa1n02x7               g142(.a(new_n237), .b(new_n236), .out0(\s[22] ));
  nor002aa1n02x5               g143(.a(\b[19] ), .b(\a[20] ), .o1(new_n239));
  nona23aa1n09x5               g144(.a(new_n216), .b(new_n213), .c(new_n212), .d(new_n239), .out0(new_n240));
  nor042aa1n06x5               g145(.a(new_n234), .b(new_n232), .o1(new_n241));
  nona23aa1n09x5               g146(.a(new_n241), .b(new_n196), .c(new_n240), .d(new_n204), .out0(new_n242));
  oai112aa1n04x5               g147(.a(new_n226), .b(new_n215), .c(new_n240), .d(new_n208), .o1(new_n243));
  inv020aa1n02x5               g148(.a(new_n231), .o1(new_n244));
  oaoi03aa1n02x5               g149(.a(\a[22] ), .b(\b[21] ), .c(new_n244), .o1(new_n245));
  aoi012aa1n02x5               g150(.a(new_n245), .b(new_n243), .c(new_n241), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n242), .c(new_n190), .d(new_n195), .o1(new_n247));
  xorb03aa1n02x5               g152(.a(new_n247), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  tech160nm_fixorc02aa1n03p5x5 g154(.a(\a[23] ), .b(\b[22] ), .out0(new_n250));
  xorc02aa1n12x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  aoi112aa1n02x5               g156(.a(new_n249), .b(new_n251), .c(new_n247), .d(new_n250), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n251), .b(new_n249), .c(new_n247), .d(new_n250), .o1(new_n253));
  norb02aa1n02x7               g158(.a(new_n253), .b(new_n252), .out0(\s[24] ));
  nanp02aa1n02x5               g159(.a(new_n251), .b(new_n250), .o1(new_n255));
  nona23aa1n06x5               g160(.a(new_n241), .b(new_n206), .c(new_n255), .d(new_n240), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n251), .b(new_n257), .out0(new_n258));
  norp02aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n260));
  nanp03aa1n03x5               g165(.a(new_n245), .b(new_n250), .c(new_n251), .o1(new_n261));
  nona22aa1n03x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .out0(new_n262));
  aoi013aa1n02x4               g167(.a(new_n262), .b(new_n243), .c(new_n241), .d(new_n258), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n263), .b(new_n256), .c(new_n190), .d(new_n195), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  xorc02aa1n12x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  aoi112aa1n02x7               g173(.a(new_n266), .b(new_n268), .c(new_n264), .d(new_n267), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n268), .b(new_n266), .c(new_n264), .d(new_n267), .o1(new_n270));
  norb02aa1n02x7               g175(.a(new_n270), .b(new_n269), .out0(\s[26] ));
  nor002aa1n02x5               g176(.a(new_n192), .b(new_n176), .o1(new_n272));
  aoi112aa1n02x5               g177(.a(new_n272), .b(new_n181), .c(new_n182), .d(new_n179), .o1(new_n273));
  oai012aa1n03x5               g178(.a(new_n273), .b(new_n155), .c(new_n188), .o1(new_n274));
  and002aa1n09x5               g179(.a(new_n268), .b(new_n267), .o(new_n275));
  nano22aa1n03x7               g180(.a(new_n242), .b(new_n258), .c(new_n275), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n274), .c(new_n167), .d(new_n189), .o1(new_n277));
  nano22aa1n03x7               g182(.a(new_n228), .b(new_n241), .c(new_n258), .out0(new_n278));
  inv000aa1d42x5               g183(.a(\a[26] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(\b[25] ), .o1(new_n280));
  oaoi03aa1n06x5               g185(.a(new_n279), .b(new_n280), .c(new_n266), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  oaoi13aa1n09x5               g187(.a(new_n282), .b(new_n275), .c(new_n278), .d(new_n262), .o1(new_n283));
  xorc02aa1n12x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnbna2aa1n03x5               g189(.a(new_n284), .b(new_n283), .c(new_n277), .out0(\s[27] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  inv040aa1n03x5               g191(.a(new_n286), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n284), .o1(new_n288));
  aoi012aa1n06x5               g193(.a(new_n288), .b(new_n283), .c(new_n277), .o1(new_n289));
  xnrc02aa1n12x5               g194(.a(\b[27] ), .b(\a[28] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n289), .b(new_n287), .c(new_n290), .out0(new_n291));
  nona32aa1n06x5               g196(.a(new_n243), .b(new_n255), .c(new_n234), .d(new_n232), .out0(new_n292));
  inv000aa1n02x5               g197(.a(new_n262), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n275), .o1(new_n294));
  aoai13aa1n06x5               g199(.a(new_n281), .b(new_n294), .c(new_n292), .d(new_n293), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n284), .b(new_n295), .c(new_n200), .d(new_n276), .o1(new_n296));
  aoi012aa1n03x5               g201(.a(new_n290), .b(new_n296), .c(new_n287), .o1(new_n297));
  norp02aa1n03x5               g202(.a(new_n297), .b(new_n291), .o1(\s[28] ));
  tech160nm_fixnrc02aa1n04x5   g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  norb02aa1d27x5               g204(.a(new_n284), .b(new_n290), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n295), .c(new_n200), .d(new_n276), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n302));
  aoi012aa1n02x7               g207(.a(new_n299), .b(new_n301), .c(new_n302), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n300), .o1(new_n304));
  tech160nm_fiaoi012aa1n05x5   g209(.a(new_n304), .b(new_n283), .c(new_n277), .o1(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n299), .c(new_n302), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n303), .b(new_n306), .o1(\s[29] ));
  xorb03aa1n02x5               g212(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g213(.a(new_n284), .b(new_n299), .c(new_n290), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n295), .c(new_n200), .d(new_n276), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .c(new_n302), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[29] ), .b(\a[30] ), .out0(new_n312));
  aoi012aa1n03x5               g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n309), .o1(new_n314));
  tech160nm_fiaoi012aa1n03p5x5 g219(.a(new_n314), .b(new_n283), .c(new_n277), .o1(new_n315));
  nano22aa1n03x7               g220(.a(new_n315), .b(new_n311), .c(new_n312), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n313), .b(new_n316), .o1(\s[30] ));
  norb02aa1n06x5               g222(.a(new_n309), .b(new_n312), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n318), .o1(new_n319));
  aoi012aa1n02x7               g224(.a(new_n319), .b(new_n283), .c(new_n277), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[30] ), .b(\a[31] ), .out0(new_n322));
  nano22aa1n03x5               g227(.a(new_n320), .b(new_n321), .c(new_n322), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n318), .b(new_n295), .c(new_n200), .d(new_n276), .o1(new_n324));
  aoi012aa1n03x5               g229(.a(new_n322), .b(new_n324), .c(new_n321), .o1(new_n325));
  norp02aa1n03x5               g230(.a(new_n325), .b(new_n323), .o1(\s[31] ));
  xnbna2aa1n03x5               g231(.a(new_n107), .b(new_n101), .c(new_n102), .out0(\s[3] ));
  oaoi03aa1n02x5               g232(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g234(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g235(.a(new_n120), .b(new_n122), .c(new_n110), .o1(new_n331));
  xorb03aa1n02x5               g236(.a(new_n331), .b(\b[5] ), .c(new_n121), .out0(\s[6] ));
  oai012aa1n02x5               g237(.a(new_n123), .b(new_n162), .c(new_n117), .o1(new_n333));
  nona23aa1n02x4               g238(.a(new_n333), .b(new_n113), .c(new_n114), .d(new_n119), .out0(new_n334));
  inv000aa1d42x5               g239(.a(new_n114), .o1(new_n335));
  aboi22aa1n03x5               g240(.a(new_n119), .b(new_n333), .c(new_n113), .d(new_n335), .out0(new_n336));
  norb02aa1n02x5               g241(.a(new_n334), .b(new_n336), .out0(\s[7] ));
  norb02aa1n02x5               g242(.a(new_n112), .b(new_n111), .out0(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n334), .c(new_n335), .out0(\s[8] ));
  xorb03aa1n02x5               g244(.a(new_n167), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


