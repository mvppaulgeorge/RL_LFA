// Benchmark "adder" written by ABC on Wed Jul 17 23:04:49 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n318, new_n320, new_n323, new_n325, new_n326, new_n328,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  and002aa1n12x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  nor042aa1n04x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n18x5               g004(.a(\b[8] ), .b(\a[9] ), .o(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[4] ), .b(\a[5] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nor002aa1d24x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nand42aa1n10x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nano23aa1n06x5               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  orn002aa1n24x5               g012(.a(\a[6] ), .b(\b[5] ), .o(new_n108));
  nand42aa1n08x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  oai112aa1n06x5               g014(.a(new_n108), .b(new_n109), .c(\b[4] ), .d(\a[5] ), .o1(new_n110));
  nanb03aa1n02x5               g015(.a(new_n110), .b(new_n107), .c(new_n102), .out0(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nand42aa1n04x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  norb03aa1n02x5               g020(.a(new_n114), .b(new_n113), .c(new_n115), .out0(new_n116));
  nor002aa1n06x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n04x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nanb03aa1n02x5               g023(.a(new_n117), .b(new_n118), .c(new_n114), .out0(new_n119));
  oab012aa1n02x4               g024(.a(new_n117), .b(\a[4] ), .c(\b[3] ), .out0(new_n120));
  oai012aa1n03x5               g025(.a(new_n120), .b(new_n116), .c(new_n119), .o1(new_n121));
  nanp02aa1n03x5               g026(.a(new_n121), .b(new_n112), .o1(new_n122));
  aoi112aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  aoi113aa1n03x7               g028(.a(new_n123), .b(new_n103), .c(new_n107), .d(new_n110), .e(new_n109), .o1(new_n124));
  oai122aa1n02x7               g029(.a(new_n124), .b(new_n122), .c(new_n111), .d(\b[8] ), .e(\a[9] ), .o1(new_n125));
  xobna2aa1n03x5               g030(.a(new_n99), .b(new_n125), .c(new_n101), .out0(\s[10] ));
  nanp02aa1n03x5               g031(.a(new_n125), .b(new_n101), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand42aa1n06x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  norb02aa1n12x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n98), .c(new_n127), .d(new_n99), .o1(new_n132));
  aoi112aa1n06x5               g037(.a(new_n131), .b(new_n98), .c(new_n127), .d(new_n99), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(new_n128), .o1(new_n135));
  nor042aa1n04x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n09x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n12x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nano22aa1n02x4               g044(.a(new_n133), .b(new_n135), .c(new_n139), .out0(new_n140));
  nanp02aa1n03x5               g045(.a(new_n127), .b(new_n99), .o1(new_n141));
  nona22aa1n02x4               g046(.a(new_n141), .b(new_n131), .c(new_n98), .out0(new_n142));
  aoi012aa1n03x5               g047(.a(new_n139), .b(new_n142), .c(new_n135), .o1(new_n143));
  norp02aa1n02x5               g048(.a(new_n143), .b(new_n140), .o1(\s[12] ));
  nor002aa1n02x5               g049(.a(\b[8] ), .b(\a[9] ), .o1(new_n145));
  tech160nm_fioai012aa1n05x5   g050(.a(new_n124), .b(new_n122), .c(new_n111), .o1(new_n146));
  nand23aa1n03x5               g051(.a(new_n99), .b(new_n130), .c(new_n138), .o1(new_n147));
  nona32aa1n03x5               g052(.a(new_n146), .b(new_n147), .c(new_n145), .d(new_n100), .out0(new_n148));
  nano23aa1n02x4               g053(.a(new_n128), .b(new_n136), .c(new_n137), .d(new_n129), .out0(new_n149));
  aob012aa1n02x5               g054(.a(new_n145), .b(\b[9] ), .c(\a[10] ), .out0(new_n150));
  tech160nm_fioai012aa1n03p5x5 g055(.a(new_n150), .b(\b[9] ), .c(\a[10] ), .o1(new_n151));
  oaoi03aa1n09x5               g056(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n152), .b(new_n149), .c(new_n151), .o1(new_n153));
  xorc02aa1n12x5               g058(.a(\a[13] ), .b(\b[12] ), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  aob012aa1n02x5               g060(.a(new_n154), .b(new_n148), .c(new_n153), .out0(new_n156));
  inv000aa1d42x5               g061(.a(\a[14] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[13] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n158), .b(new_n157), .o1(new_n159));
  tech160nm_fixorc02aa1n04x5   g064(.a(\a[14] ), .b(\b[13] ), .out0(new_n160));
  nand22aa1n03x5               g065(.a(new_n160), .b(new_n154), .o1(new_n161));
  oai022aa1n02x5               g066(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n162));
  oaib12aa1n02x5               g067(.a(new_n162), .b(new_n158), .c(\a[14] ), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n161), .c(new_n148), .d(new_n153), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\a[13] ), .o1(new_n165));
  aoib12aa1n02x5               g070(.a(new_n160), .b(new_n165), .c(\b[12] ), .out0(new_n166));
  aoi022aa1n02x5               g071(.a(new_n159), .b(new_n164), .c(new_n156), .d(new_n166), .o1(\s[14] ));
  xorb03aa1n02x5               g072(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  xnrc02aa1n12x5               g073(.a(\b[14] ), .b(\a[15] ), .out0(new_n169));
  nanb02aa1n03x5               g074(.a(new_n169), .b(new_n164), .out0(new_n170));
  nor022aa1n08x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n20x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  aoib12aa1n02x5               g078(.a(new_n172), .b(new_n173), .c(new_n171), .out0(new_n174));
  nanb02aa1n02x5               g079(.a(new_n103), .b(new_n104), .out0(new_n175));
  norb02aa1n03x5               g080(.a(new_n106), .b(new_n105), .out0(new_n176));
  nano23aa1n03x7               g081(.a(new_n175), .b(new_n110), .c(new_n176), .d(new_n102), .out0(new_n177));
  norb02aa1n06x5               g082(.a(new_n118), .b(new_n117), .out0(new_n178));
  oai112aa1n04x5               g083(.a(new_n178), .b(new_n114), .c(new_n115), .d(new_n113), .o1(new_n179));
  aoi022aa1n06x5               g084(.a(new_n179), .b(new_n120), .c(\b[3] ), .d(\a[4] ), .o1(new_n180));
  nanp03aa1n02x5               g085(.a(new_n107), .b(new_n109), .c(new_n110), .o1(new_n181));
  nona22aa1n03x5               g086(.a(new_n181), .b(new_n123), .c(new_n103), .out0(new_n182));
  norb02aa1n06x5               g087(.a(new_n173), .b(new_n171), .out0(new_n183));
  nanb02aa1d24x5               g088(.a(new_n169), .b(new_n183), .out0(new_n184));
  nona23aa1n03x5               g089(.a(new_n154), .b(new_n160), .c(new_n145), .d(new_n100), .out0(new_n185));
  nor043aa1n03x5               g090(.a(new_n185), .b(new_n184), .c(new_n147), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n186), .b(new_n182), .c(new_n180), .d(new_n177), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n184), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n189));
  oai112aa1n03x5               g094(.a(new_n138), .b(new_n130), .c(new_n189), .d(new_n97), .o1(new_n190));
  inv000aa1n02x5               g095(.a(new_n152), .o1(new_n191));
  aoai13aa1n06x5               g096(.a(new_n163), .b(new_n161), .c(new_n190), .d(new_n191), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n173), .b(new_n171), .c(new_n172), .o1(new_n193));
  aobi12aa1n12x5               g098(.a(new_n193), .b(new_n192), .c(new_n188), .out0(new_n194));
  nand02aa1d06x5               g099(.a(new_n194), .b(new_n187), .o1(new_n195));
  aboi22aa1n03x5               g100(.a(new_n171), .b(new_n195), .c(new_n170), .d(new_n174), .out0(\s[16] ));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n194), .c(new_n187), .out0(\s[17] ));
  nor042aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  inv040aa1d30x5               g104(.a(\a[17] ), .o1(new_n200));
  inv040aa1d32x5               g105(.a(\a[18] ), .o1(new_n201));
  xroi22aa1d06x4               g106(.a(new_n200), .b(\b[16] ), .c(new_n201), .d(\b[17] ), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  inv000aa1d42x5               g108(.a(\b[16] ), .o1(new_n204));
  nand22aa1n04x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  aoi013aa1n06x4               g110(.a(new_n199), .b(new_n205), .c(new_n200), .d(new_n204), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n203), .c(new_n194), .d(new_n187), .o1(new_n207));
  obai22aa1n02x7               g112(.a(new_n205), .b(new_n199), .c(\a[17] ), .d(\b[16] ), .out0(new_n208));
  aoi012aa1n02x5               g113(.a(new_n208), .b(new_n195), .c(new_n197), .o1(new_n209));
  aoib12aa1n02x5               g114(.a(new_n209), .b(new_n207), .c(new_n199), .out0(\s[18] ));
  xorb03aa1n03x5               g115(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nanp02aa1n04x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nor042aa1n06x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanp02aa1n04x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  aoi112aa1n03x4               g122(.a(new_n213), .b(new_n217), .c(new_n207), .d(new_n214), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n217), .b(new_n213), .c(new_n207), .d(new_n214), .o1(new_n219));
  norb02aa1n02x7               g124(.a(new_n219), .b(new_n218), .out0(\s[20] ));
  nano23aa1n06x5               g125(.a(new_n213), .b(new_n215), .c(new_n216), .d(new_n214), .out0(new_n221));
  nand02aa1n02x5               g126(.a(new_n202), .b(new_n221), .o1(new_n222));
  nona23aa1n09x5               g127(.a(new_n216), .b(new_n214), .c(new_n213), .d(new_n215), .out0(new_n223));
  tech160nm_fioai012aa1n03p5x5 g128(.a(new_n216), .b(new_n215), .c(new_n213), .o1(new_n224));
  oai012aa1n12x5               g129(.a(new_n224), .b(new_n223), .c(new_n206), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n04x5               g131(.a(new_n226), .b(new_n222), .c(new_n194), .d(new_n187), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n02x7               g138(.a(new_n233), .b(new_n232), .out0(\s[22] ));
  inv000aa1d42x5               g139(.a(\a[21] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\a[22] ), .o1(new_n236));
  xroi22aa1d04x5               g141(.a(new_n235), .b(\b[20] ), .c(new_n236), .d(\b[21] ), .out0(new_n237));
  nand23aa1n06x5               g142(.a(new_n237), .b(new_n202), .c(new_n221), .o1(new_n238));
  nona22aa1n02x4               g143(.a(new_n205), .b(\b[16] ), .c(\a[17] ), .out0(new_n239));
  oaib12aa1n02x5               g144(.a(new_n239), .b(\b[17] ), .c(new_n201), .out0(new_n240));
  inv040aa1n03x5               g145(.a(new_n224), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n237), .b(new_n241), .c(new_n221), .d(new_n240), .o1(new_n242));
  inv000aa1d42x5               g147(.a(\b[21] ), .o1(new_n243));
  oao003aa1n06x5               g148(.a(new_n236), .b(new_n243), .c(new_n229), .carry(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  nand22aa1n03x5               g150(.a(new_n242), .b(new_n245), .o1(new_n246));
  inv000aa1n02x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n04x5               g152(.a(new_n247), .b(new_n238), .c(new_n194), .d(new_n187), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  tech160nm_fixorc02aa1n02p5x5 g155(.a(\a[23] ), .b(\b[22] ), .out0(new_n251));
  xorc02aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .out0(new_n252));
  aoi112aa1n02x5               g157(.a(new_n250), .b(new_n252), .c(new_n248), .d(new_n251), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n252), .b(new_n250), .c(new_n248), .d(new_n251), .o1(new_n254));
  norb02aa1n02x7               g159(.a(new_n254), .b(new_n253), .out0(\s[24] ));
  and002aa1n06x5               g160(.a(new_n252), .b(new_n251), .o(new_n256));
  nanb03aa1n03x5               g161(.a(new_n222), .b(new_n256), .c(new_n237), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n256), .o1(new_n258));
  oai022aa1n02x5               g163(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n259));
  aob012aa1n02x5               g164(.a(new_n259), .b(\b[23] ), .c(\a[24] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n258), .c(new_n242), .d(new_n245), .o1(new_n261));
  inv040aa1n03x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n257), .c(new_n194), .d(new_n187), .o1(new_n263));
  xorb03aa1n02x5               g168(.a(new_n263), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  tech160nm_fixorc02aa1n05x5   g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  xorc02aa1n03x5               g171(.a(\a[26] ), .b(\b[25] ), .out0(new_n267));
  aoi112aa1n02x5               g172(.a(new_n265), .b(new_n267), .c(new_n263), .d(new_n266), .o1(new_n268));
  aoai13aa1n03x5               g173(.a(new_n267), .b(new_n265), .c(new_n263), .d(new_n266), .o1(new_n269));
  norb02aa1n02x7               g174(.a(new_n269), .b(new_n268), .out0(\s[26] ));
  xroi22aa1d04x5               g175(.a(new_n165), .b(\b[12] ), .c(new_n157), .d(\b[13] ), .out0(new_n271));
  aoai13aa1n02x5               g176(.a(new_n271), .b(new_n152), .c(new_n149), .d(new_n151), .o1(new_n272));
  aoai13aa1n02x5               g177(.a(new_n193), .b(new_n184), .c(new_n272), .d(new_n163), .o1(new_n273));
  and002aa1n06x5               g178(.a(new_n267), .b(new_n266), .o(new_n274));
  nano22aa1n12x5               g179(.a(new_n238), .b(new_n256), .c(new_n274), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n273), .c(new_n146), .d(new_n186), .o1(new_n276));
  oai022aa1n02x5               g181(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n277));
  aob012aa1n02x5               g182(.a(new_n277), .b(\b[25] ), .c(\a[26] ), .out0(new_n278));
  aobi12aa1n06x5               g183(.a(new_n278), .b(new_n261), .c(new_n274), .out0(new_n279));
  xorc02aa1n02x5               g184(.a(\a[27] ), .b(\b[26] ), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n276), .c(new_n279), .out0(\s[27] ));
  nor042aa1n03x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  aobi12aa1n02x7               g188(.a(new_n280), .b(new_n276), .c(new_n279), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  aoai13aa1n02x7               g191(.a(new_n256), .b(new_n244), .c(new_n225), .d(new_n237), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n274), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n278), .b(new_n288), .c(new_n287), .d(new_n260), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n280), .b(new_n289), .c(new_n195), .d(new_n275), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n285), .b(new_n290), .c(new_n283), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  norb02aa1n02x5               g198(.a(new_n280), .b(new_n285), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n289), .c(new_n195), .d(new_n275), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n293), .b(new_n295), .c(new_n296), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n294), .b(new_n276), .c(new_n279), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n280), .b(new_n293), .c(new_n285), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n289), .c(new_n195), .d(new_n275), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x7               g211(.a(new_n302), .b(new_n276), .c(new_n279), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  norb02aa1n02x5               g214(.a(new_n302), .b(new_n305), .out0(new_n310));
  aobi12aa1n02x7               g215(.a(new_n310), .b(new_n276), .c(new_n279), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[30] ), .b(\a[31] ), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n313), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n310), .b(new_n289), .c(new_n195), .d(new_n275), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n313), .b(new_n315), .c(new_n312), .o1(new_n316));
  norp02aa1n03x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  oaoi13aa1n02x5               g222(.a(new_n178), .b(new_n114), .c(new_n113), .d(new_n115), .o1(new_n318));
  norb02aa1n02x5               g223(.a(new_n179), .b(new_n318), .out0(\s[3] ));
  oabi12aa1n02x5               g224(.a(new_n117), .b(new_n116), .c(new_n119), .out0(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n180), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n03x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n122), .carry(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n323), .b(new_n108), .c(new_n109), .out0(\s[6] ));
  xorc02aa1n02x5               g229(.a(\a[6] ), .b(\b[5] ), .out0(new_n325));
  nanp02aa1n03x5               g230(.a(new_n323), .b(new_n325), .o1(new_n326));
  xobna2aa1n03x5               g231(.a(new_n176), .b(new_n326), .c(new_n109), .out0(\s[7] ));
  inv000aa1d42x5               g232(.a(new_n105), .o1(new_n328));
  nanp03aa1n02x5               g233(.a(new_n326), .b(new_n176), .c(new_n109), .o1(new_n329));
  xobna2aa1n03x5               g234(.a(new_n175), .b(new_n329), .c(new_n328), .out0(\s[8] ));
  xorb03aa1n02x5               g235(.a(new_n146), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


