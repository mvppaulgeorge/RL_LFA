// Benchmark "adder" written by ABC on Thu Jul 18 09:33:13 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n343, new_n344, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nand42aa1n20x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  inv000aa1n02x5               g005(.a(new_n100), .o1(new_n101));
  nor002aa1n03x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  oab012aa1n03x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .out0(new_n104));
  norb02aa1n06x4               g009(.a(new_n100), .b(new_n102), .out0(new_n105));
  oaih22aa1n06x5               g010(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nanp02aa1n06x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nano22aa1n03x7               g013(.a(new_n103), .b(new_n107), .c(new_n108), .out0(new_n109));
  aoi013aa1n06x4               g014(.a(new_n104), .b(new_n109), .c(new_n106), .d(new_n105), .o1(new_n110));
  nor042aa1n06x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  inv040aa1n03x5               g016(.a(new_n111), .o1(new_n112));
  oao003aa1n03x5               g017(.a(\a[4] ), .b(\b[3] ), .c(new_n112), .carry(new_n113));
  tech160nm_fixorc02aa1n04x5   g018(.a(\a[4] ), .b(\b[3] ), .out0(new_n114));
  nanp02aa1n04x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand02aa1d16x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nano22aa1n02x4               g021(.a(new_n111), .b(new_n115), .c(new_n116), .out0(new_n117));
  nand02aa1d04x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nor042aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  nona22aa1n02x4               g024(.a(new_n115), .b(new_n119), .c(new_n118), .out0(new_n120));
  nanp03aa1n03x5               g025(.a(new_n117), .b(new_n120), .c(new_n114), .o1(new_n121));
  nand42aa1n03x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nano22aa1n03x7               g027(.a(new_n106), .b(new_n107), .c(new_n122), .out0(new_n123));
  oaih22aa1n04x5               g028(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  nor042aa1n02x5               g029(.a(new_n124), .b(new_n101), .o1(new_n125));
  nand23aa1n04x5               g030(.a(new_n123), .b(new_n125), .c(new_n108), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n110), .b(new_n126), .c(new_n121), .d(new_n113), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n127), .c(new_n99), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  aoi112aa1n06x5               g035(.a(new_n130), .b(new_n98), .c(new_n127), .d(new_n99), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\a[11] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(\b[10] ), .o1(new_n133));
  nand02aa1d08x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  nanp02aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand22aa1n09x5               g040(.a(new_n134), .b(new_n135), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n131), .c(\a[10] ), .d(\b[9] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[9] ), .b(\a[10] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n136), .o1(new_n139));
  nano22aa1n03x7               g044(.a(new_n131), .b(new_n138), .c(new_n139), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n137), .b(new_n140), .out0(\s[11] ));
  norp02aa1n04x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanb02aa1n06x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  aoib12aa1n02x5               g049(.a(new_n144), .b(new_n134), .c(new_n140), .out0(new_n145));
  nano22aa1n02x4               g050(.a(new_n140), .b(new_n134), .c(new_n144), .out0(new_n146));
  norp02aa1n02x5               g051(.a(new_n145), .b(new_n146), .o1(\s[12] ));
  inv000aa1d42x5               g052(.a(\a[13] ), .o1(new_n148));
  nanb02aa1n06x5               g053(.a(new_n98), .b(new_n99), .out0(new_n149));
  nor042aa1n06x5               g054(.a(new_n144), .b(new_n136), .o1(new_n150));
  nona22aa1d30x5               g055(.a(new_n150), .b(new_n130), .c(new_n149), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n143), .b(new_n142), .c(new_n132), .d(new_n133), .o1(new_n153));
  inv000aa1d42x5               g058(.a(\b[9] ), .o1(new_n154));
  oaoi03aa1n02x5               g059(.a(new_n97), .b(new_n154), .c(new_n98), .o1(new_n155));
  oai013aa1n03x4               g060(.a(new_n153), .b(new_n155), .c(new_n136), .d(new_n144), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n156), .b(new_n127), .c(new_n152), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(new_n148), .out0(\s[13] ));
  nanb02aa1d24x5               g063(.a(\b[12] ), .b(new_n148), .out0(new_n159));
  tech160nm_fixorc02aa1n02p5x5 g064(.a(\a[13] ), .b(\b[12] ), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n156), .c(new_n127), .d(new_n152), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n159), .out0(\s[14] ));
  xnrc02aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .out0(new_n164));
  norb02aa1n02x5               g069(.a(new_n160), .b(new_n164), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n156), .c(new_n127), .d(new_n152), .o1(new_n166));
  oaoi03aa1n12x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .o1(new_n167));
  inv030aa1n02x5               g072(.a(new_n167), .o1(new_n168));
  xorc02aa1n12x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n168), .out0(\s[15] ));
  nor042aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  xnrc02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .out0(new_n173));
  aoi012aa1n02x5               g078(.a(new_n173), .b(new_n166), .c(new_n168), .o1(new_n174));
  nor022aa1n06x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n03x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanb02aa1n06x5               g081(.a(new_n175), .b(new_n176), .out0(new_n177));
  nano22aa1n02x4               g082(.a(new_n174), .b(new_n172), .c(new_n177), .out0(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[3] ), .b(\a[4] ), .out0(new_n179));
  norb02aa1n06x5               g084(.a(new_n116), .b(new_n111), .out0(new_n180));
  oai112aa1n03x5               g085(.a(new_n180), .b(new_n115), .c(new_n118), .d(new_n119), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n113), .b(new_n181), .c(new_n179), .o1(new_n182));
  inv000aa1n02x5               g087(.a(new_n126), .o1(new_n183));
  nand42aa1n02x5               g088(.a(new_n182), .b(new_n183), .o1(new_n184));
  oao003aa1n02x5               g089(.a(new_n97), .b(new_n154), .c(new_n98), .carry(new_n185));
  aobi12aa1n06x5               g090(.a(new_n153), .b(new_n150), .c(new_n185), .out0(new_n186));
  aoai13aa1n02x5               g091(.a(new_n186), .b(new_n151), .c(new_n184), .d(new_n110), .o1(new_n187));
  aoai13aa1n02x5               g092(.a(new_n169), .b(new_n167), .c(new_n187), .d(new_n165), .o1(new_n188));
  aoi012aa1n02x5               g093(.a(new_n177), .b(new_n188), .c(new_n172), .o1(new_n189));
  norp02aa1n02x5               g094(.a(new_n189), .b(new_n178), .o1(\s[16] ));
  nanb02aa1n03x5               g095(.a(new_n177), .b(new_n169), .out0(new_n191));
  nano22aa1n02x4               g096(.a(new_n191), .b(new_n160), .c(new_n162), .out0(new_n192));
  nanb02aa1n03x5               g097(.a(new_n151), .b(new_n192), .out0(new_n193));
  oaih12aa1n02x5               g098(.a(new_n176), .b(new_n175), .c(new_n171), .o1(new_n194));
  oai012aa1n02x5               g099(.a(new_n194), .b(new_n191), .c(new_n168), .o1(new_n195));
  aoi012aa1n02x7               g100(.a(new_n195), .b(new_n156), .c(new_n192), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(new_n184), .d(new_n110), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g103(.a(\a[18] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[16] ), .o1(new_n201));
  oaoi03aa1n03x5               g106(.a(new_n200), .b(new_n201), .c(new_n197), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(new_n199), .out0(\s[18] ));
  norp02aa1n02x5               g108(.a(new_n173), .b(new_n177), .o1(new_n204));
  nano32aa1n03x7               g109(.a(new_n151), .b(new_n204), .c(new_n160), .d(new_n162), .out0(new_n205));
  nona23aa1n02x4               g110(.a(new_n160), .b(new_n169), .c(new_n164), .d(new_n177), .out0(new_n206));
  aobi12aa1n02x5               g111(.a(new_n194), .b(new_n204), .c(new_n167), .out0(new_n207));
  oai012aa1n06x5               g112(.a(new_n207), .b(new_n186), .c(new_n206), .o1(new_n208));
  xroi22aa1d06x4               g113(.a(new_n200), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n208), .c(new_n127), .d(new_n205), .o1(new_n210));
  inv000aa1d42x5               g115(.a(\b[17] ), .o1(new_n211));
  oaih22aa1n04x5               g116(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n212));
  oa0012aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n199), .o(new_n213));
  inv000aa1d42x5               g118(.a(\a[19] ), .o1(new_n214));
  inv000aa1d42x5               g119(.a(\b[18] ), .o1(new_n215));
  nand22aa1n06x5               g120(.a(new_n215), .b(new_n214), .o1(new_n216));
  nand42aa1n16x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nand02aa1d04x5               g122(.a(new_n216), .b(new_n217), .o1(new_n218));
  aoib12aa1n06x5               g123(.a(new_n218), .b(new_n210), .c(new_n213), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n218), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n210), .b(new_n213), .c(new_n220), .out0(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand02aa1d16x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  nano22aa1n02x4               g132(.a(new_n219), .b(new_n216), .c(new_n227), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n220), .b(new_n213), .c(new_n197), .d(new_n209), .o1(new_n229));
  aoi012aa1n03x5               g134(.a(new_n227), .b(new_n229), .c(new_n216), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n228), .o1(\s[20] ));
  nona22aa1d18x5               g136(.a(new_n209), .b(new_n218), .c(new_n227), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n208), .c(new_n127), .d(new_n205), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n225), .b(new_n224), .c(new_n214), .d(new_n215), .o1(new_n235));
  oai112aa1n04x5               g140(.a(new_n216), .b(new_n217), .c(new_n211), .d(new_n199), .o1(new_n236));
  inv000aa1d42x5               g141(.a(\b[19] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(\a[20] ), .b(new_n237), .out0(new_n238));
  nand23aa1n04x5               g143(.a(new_n212), .b(new_n238), .c(new_n225), .o1(new_n239));
  oai012aa1n12x5               g144(.a(new_n235), .b(new_n239), .c(new_n236), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[20] ), .b(\a[21] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnbna2aa1n03x5               g148(.a(new_n243), .b(new_n234), .c(new_n241), .out0(\s[21] ));
  orn002aa1n24x5               g149(.a(\a[21] ), .b(\b[20] ), .o(new_n245));
  tech160nm_fiaoi012aa1n03p5x5 g150(.a(new_n242), .b(new_n234), .c(new_n241), .o1(new_n246));
  tech160nm_fixnrc02aa1n02p5x5 g151(.a(\b[21] ), .b(\a[22] ), .out0(new_n247));
  nano22aa1n02x4               g152(.a(new_n246), .b(new_n245), .c(new_n247), .out0(new_n248));
  aoai13aa1n03x5               g153(.a(new_n243), .b(new_n240), .c(new_n197), .d(new_n233), .o1(new_n249));
  aoi012aa1n03x5               g154(.a(new_n247), .b(new_n249), .c(new_n245), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n248), .o1(\s[22] ));
  nor042aa1n06x5               g156(.a(new_n247), .b(new_n242), .o1(new_n252));
  inv040aa1n08x5               g157(.a(new_n252), .o1(new_n253));
  nor002aa1n02x5               g158(.a(new_n232), .b(new_n253), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n208), .c(new_n127), .d(new_n205), .o1(new_n255));
  norp02aa1n02x5               g160(.a(\b[18] ), .b(\a[19] ), .o1(new_n256));
  inv020aa1n04x5               g161(.a(new_n217), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n257), .b(new_n256), .c(\a[18] ), .d(\b[17] ), .o1(new_n258));
  nand03aa1n02x5               g163(.a(new_n258), .b(new_n212), .c(new_n226), .o1(new_n259));
  oaoi03aa1n09x5               g164(.a(\a[22] ), .b(\b[21] ), .c(new_n245), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n12x5               g166(.a(new_n261), .b(new_n253), .c(new_n259), .d(new_n235), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  tech160nm_fixorc02aa1n03p5x5 g168(.a(\a[23] ), .b(\b[22] ), .out0(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n255), .c(new_n263), .out0(\s[23] ));
  orn002aa1n12x5               g170(.a(\a[23] ), .b(\b[22] ), .o(new_n266));
  aobi12aa1n06x5               g171(.a(new_n264), .b(new_n255), .c(new_n263), .out0(new_n267));
  xorc02aa1n12x5               g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  nano22aa1n02x4               g174(.a(new_n267), .b(new_n266), .c(new_n269), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n264), .b(new_n262), .c(new_n197), .d(new_n254), .o1(new_n271));
  aoi012aa1n03x5               g176(.a(new_n269), .b(new_n271), .c(new_n266), .o1(new_n272));
  norp02aa1n03x5               g177(.a(new_n272), .b(new_n270), .o1(\s[24] ));
  oaoi03aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .o1(new_n274));
  and002aa1n06x5               g179(.a(new_n268), .b(new_n264), .o(new_n275));
  tech160nm_fiaoi012aa1n05x5   g180(.a(new_n274), .b(new_n262), .c(new_n275), .o1(new_n276));
  nano32aa1n03x7               g181(.a(new_n232), .b(new_n268), .c(new_n252), .d(new_n264), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n208), .c(new_n127), .d(new_n205), .o1(new_n278));
  xnrc02aa1n12x5               g183(.a(\b[24] ), .b(\a[25] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n278), .c(new_n276), .out0(\s[25] ));
  nor042aa1n03x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  aoi012aa1n03x5               g188(.a(new_n279), .b(new_n278), .c(new_n276), .o1(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[25] ), .b(\a[26] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  inv000aa1n06x5               g191(.a(new_n276), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n280), .b(new_n287), .c(new_n197), .d(new_n277), .o1(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n285), .b(new_n288), .c(new_n283), .o1(new_n289));
  nor002aa1n02x5               g194(.a(new_n289), .b(new_n286), .o1(\s[26] ));
  nor042aa1n02x5               g195(.a(new_n285), .b(new_n279), .o1(new_n291));
  inv000aa1n02x5               g196(.a(new_n291), .o1(new_n292));
  nano23aa1d12x5               g197(.a(new_n232), .b(new_n292), .c(new_n275), .d(new_n252), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n208), .c(new_n127), .d(new_n205), .o1(new_n294));
  aoai13aa1n09x5               g199(.a(new_n291), .b(new_n274), .c(new_n262), .d(new_n275), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .carry(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoi013aa1n03x5               g203(.a(new_n298), .b(new_n294), .c(new_n295), .d(new_n296), .o1(new_n299));
  inv000aa1n02x5               g204(.a(new_n274), .o1(new_n300));
  aoai13aa1n06x5               g205(.a(new_n275), .b(new_n260), .c(new_n240), .d(new_n252), .o1(new_n301));
  aoai13aa1n04x5               g206(.a(new_n296), .b(new_n292), .c(new_n301), .d(new_n300), .o1(new_n302));
  aoi112aa1n02x5               g207(.a(new_n302), .b(new_n297), .c(new_n197), .d(new_n293), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n299), .b(new_n303), .o1(\s[27] ));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  inv040aa1n03x5               g210(.a(new_n305), .o1(new_n306));
  tech160nm_fixnrc02aa1n05x5   g211(.a(\b[27] ), .b(\a[28] ), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n299), .b(new_n306), .c(new_n307), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n297), .b(new_n302), .c(new_n197), .d(new_n293), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[28] ));
  xnrc02aa1n02x5               g216(.a(\b[28] ), .b(\a[29] ), .out0(new_n312));
  norb02aa1n06x5               g217(.a(new_n297), .b(new_n307), .out0(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  aoi013aa1n03x5               g219(.a(new_n314), .b(new_n294), .c(new_n295), .d(new_n296), .o1(new_n315));
  oao003aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .carry(new_n316));
  nano22aa1n03x5               g221(.a(new_n315), .b(new_n312), .c(new_n316), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n313), .b(new_n302), .c(new_n197), .d(new_n293), .o1(new_n318));
  tech160nm_fiaoi012aa1n02p5x5 g223(.a(new_n312), .b(new_n318), .c(new_n316), .o1(new_n319));
  norp02aa1n03x5               g224(.a(new_n319), .b(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g226(.a(new_n297), .b(new_n312), .c(new_n307), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n322), .o1(new_n323));
  aoi013aa1n02x5               g228(.a(new_n323), .b(new_n294), .c(new_n295), .d(new_n296), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n325));
  xnrc02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .out0(new_n326));
  nano22aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n326), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n322), .b(new_n302), .c(new_n197), .d(new_n293), .o1(new_n328));
  tech160nm_fiaoi012aa1n02p5x5 g233(.a(new_n326), .b(new_n328), .c(new_n325), .o1(new_n329));
  norp02aa1n03x5               g234(.a(new_n329), .b(new_n327), .o1(\s[30] ));
  norb02aa1n02x5               g235(.a(new_n322), .b(new_n326), .out0(new_n331));
  inv000aa1n02x5               g236(.a(new_n331), .o1(new_n332));
  aoi013aa1n02x5               g237(.a(new_n332), .b(new_n294), .c(new_n295), .d(new_n296), .o1(new_n333));
  oao003aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n334));
  xnrc02aa1n02x5               g239(.a(\b[30] ), .b(\a[31] ), .out0(new_n335));
  nano22aa1n03x5               g240(.a(new_n333), .b(new_n334), .c(new_n335), .out0(new_n336));
  aoai13aa1n02x5               g241(.a(new_n331), .b(new_n302), .c(new_n197), .d(new_n293), .o1(new_n337));
  tech160nm_fiaoi012aa1n02p5x5 g242(.a(new_n335), .b(new_n337), .c(new_n334), .o1(new_n338));
  norp02aa1n03x5               g243(.a(new_n338), .b(new_n336), .o1(\s[31] ));
  xobna2aa1n03x5               g244(.a(new_n180), .b(new_n120), .c(new_n115), .out0(\s[3] ));
  xnbna2aa1n03x5               g245(.a(new_n114), .b(new_n181), .c(new_n112), .out0(\s[4] ));
  xorb03aa1n02x5               g246(.a(new_n182), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  xorc02aa1n02x5               g247(.a(\a[6] ), .b(\b[5] ), .out0(new_n343));
  norp02aa1n02x5               g248(.a(\b[4] ), .b(\a[5] ), .o1(new_n344));
  aoi012aa1n02x5               g249(.a(new_n344), .b(new_n182), .c(new_n122), .o1(new_n345));
  xnrc02aa1n02x5               g250(.a(new_n345), .b(new_n343), .out0(\s[6] ));
  oaoi03aa1n02x5               g251(.a(\a[6] ), .b(\b[5] ), .c(new_n345), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g253(.a(new_n103), .b(new_n347), .c(new_n108), .o1(new_n349));
  xnrc02aa1n02x5               g254(.a(new_n349), .b(new_n105), .out0(\s[8] ));
  xorb03aa1n02x5               g255(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


