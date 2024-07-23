// Benchmark "adder" written by ABC on Thu Jul 18 05:40:02 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n325, new_n326, new_n327, new_n329, new_n330, new_n332,
    new_n333, new_n335, new_n336, new_n337, new_n339, new_n341, new_n343,
    new_n344, new_n345;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n03x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1n04x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  aoi012aa1n09x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand42aa1n06x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fiaoi012aa1n03p5x5 g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  xnrc02aa1n12x5               g012(.a(\b[7] ), .b(\a[8] ), .out0(new_n108));
  nor002aa1d32x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand02aa1d16x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  norb02aa1n06x5               g015(.a(new_n110), .b(new_n109), .out0(new_n111));
  inv000aa1n02x5               g016(.a(new_n111), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand42aa1n10x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nor002aa1d32x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n03x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nor043aa1n06x5               g022(.a(new_n117), .b(new_n112), .c(new_n108), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(new_n107), .b(new_n118), .o1(new_n119));
  nanb03aa1n03x5               g024(.a(new_n109), .b(new_n114), .c(new_n110), .out0(new_n120));
  nor042aa1n02x5               g025(.a(new_n115), .b(new_n113), .o1(new_n121));
  norp03aa1n02x5               g026(.a(new_n120), .b(new_n108), .c(new_n121), .o1(new_n122));
  inv040aa1n08x5               g027(.a(new_n109), .o1(new_n123));
  oaoi03aa1n09x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  nor042aa1n04x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nona32aa1n06x5               g030(.a(new_n119), .b(new_n125), .c(new_n124), .d(new_n122), .out0(new_n126));
  nand42aa1d28x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1d28x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nor042aa1n03x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nano22aa1n02x4               g034(.a(new_n129), .b(new_n127), .c(new_n128), .out0(new_n130));
  aboi22aa1n03x5               g035(.a(new_n129), .b(new_n127), .c(new_n126), .d(new_n128), .out0(new_n131));
  aoi012aa1n02x5               g036(.a(new_n131), .b(new_n126), .c(new_n130), .o1(\s[10] ));
  tech160nm_fiao0012aa1n02p5x5 g037(.a(new_n129), .b(new_n125), .c(new_n127), .o(new_n133));
  nor042aa1d18x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n16x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n15x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n06x5               g041(.a(new_n136), .b(new_n133), .c(new_n126), .d(new_n130), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n126), .d(new_n130), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n134), .o1(new_n140));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n12x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  inv040aa1n02x5               g049(.a(new_n124), .o1(new_n145));
  oai013aa1n03x5               g050(.a(new_n145), .b(new_n120), .c(new_n108), .d(new_n121), .o1(new_n146));
  oai012aa1d24x5               g051(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .o1(new_n147));
  oai012aa1d24x5               g052(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .o1(new_n148));
  nona23aa1d24x5               g053(.a(new_n143), .b(new_n136), .c(new_n148), .d(new_n147), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n146), .c(new_n107), .d(new_n118), .o1(new_n151));
  aoai13aa1n02x5               g056(.a(new_n135), .b(new_n129), .c(new_n125), .d(new_n127), .o1(new_n152));
  nona22aa1n02x4               g057(.a(new_n152), .b(new_n141), .c(new_n134), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n142), .o1(new_n154));
  nor042aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n10x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n151), .c(new_n154), .out0(\s[13] ));
  nanp02aa1n06x5               g063(.a(new_n151), .b(new_n154), .o1(new_n159));
  inv040aa1n02x5               g064(.a(new_n155), .o1(new_n160));
  nor042aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n16x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  oaib12aa1n02x5               g067(.a(new_n160), .b(new_n161), .c(new_n162), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n157), .o1(new_n164));
  nano23aa1n06x5               g069(.a(new_n155), .b(new_n161), .c(new_n162), .d(new_n156), .out0(new_n165));
  nanp02aa1n02x5               g070(.a(new_n159), .b(new_n165), .o1(new_n166));
  tech160nm_fioaoi03aa1n03p5x5 g071(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n161), .b(new_n166), .c(new_n168), .o1(new_n169));
  norp02aa1n02x5               g074(.a(new_n169), .b(new_n164), .o1(\s[14] ));
  nor042aa1n06x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n166), .c(new_n168), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n171), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n173), .b(new_n167), .c(new_n159), .d(new_n165), .o1(new_n176));
  nor022aa1n08x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand42aa1d28x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n178), .o1(new_n180));
  oai022aa1n02x5               g085(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n181));
  nona22aa1n02x4               g086(.a(new_n176), .b(new_n180), .c(new_n181), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n179), .c(new_n176), .d(new_n175), .o1(\s[16] ));
  nano23aa1n06x5               g088(.a(new_n171), .b(new_n177), .c(new_n178), .d(new_n172), .out0(new_n184));
  nano22aa1d15x5               g089(.a(new_n149), .b(new_n165), .c(new_n184), .out0(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n146), .c(new_n107), .d(new_n118), .o1(new_n186));
  nano22aa1n03x5               g091(.a(new_n177), .b(new_n172), .c(new_n178), .out0(new_n187));
  tech160nm_fioai012aa1n03p5x5 g092(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .o1(new_n188));
  oai012aa1n02x5               g093(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .o1(new_n189));
  nano23aa1n03x7               g094(.a(new_n189), .b(new_n188), .c(new_n160), .d(new_n142), .out0(new_n190));
  oab012aa1n02x5               g095(.a(new_n188), .b(new_n155), .c(new_n161), .out0(new_n191));
  ao0022aa1n03x7               g096(.a(new_n191), .b(new_n187), .c(new_n181), .d(new_n178), .o(new_n192));
  aoi013aa1n06x4               g097(.a(new_n192), .b(new_n153), .c(new_n187), .d(new_n190), .o1(new_n193));
  nanp02aa1n09x5               g098(.a(new_n186), .b(new_n193), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n184), .b(new_n167), .c(new_n159), .d(new_n165), .o1(new_n196));
  aoi012aa1n02x5               g101(.a(new_n195), .b(new_n178), .c(new_n181), .o1(new_n197));
  aoi022aa1n02x5               g102(.a(new_n196), .b(new_n197), .c(new_n195), .d(new_n194), .o1(\s[17] ));
  nor042aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand02aa1d16x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  obai22aa1n02x7               g105(.a(new_n200), .b(new_n199), .c(\a[17] ), .d(\b[16] ), .out0(new_n201));
  aoi012aa1n02x5               g106(.a(new_n201), .b(new_n194), .c(new_n195), .o1(new_n202));
  nanb03aa1d24x5               g107(.a(new_n199), .b(new_n195), .c(new_n200), .out0(new_n203));
  oaih22aa1d12x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  and002aa1n02x5               g109(.a(new_n204), .b(new_n200), .o(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n04x5               g111(.a(new_n206), .b(new_n203), .c(new_n186), .d(new_n193), .o1(new_n207));
  aoib12aa1n02x5               g112(.a(new_n202), .b(new_n207), .c(new_n199), .out0(\s[18] ));
  tech160nm_fixorc02aa1n03p5x5 g113(.a(\a[19] ), .b(\b[18] ), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n203), .o1(new_n210));
  aoi112aa1n02x5               g115(.a(new_n209), .b(new_n205), .c(new_n194), .d(new_n210), .o1(new_n211));
  aoi012aa1n02x5               g116(.a(new_n211), .b(new_n207), .c(new_n209), .o1(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g118(.a(\a[19] ), .o1(new_n214));
  inv000aa1d42x5               g119(.a(\b[18] ), .o1(new_n215));
  nanp02aa1n04x5               g120(.a(new_n215), .b(new_n214), .o1(new_n216));
  nanp02aa1n03x5               g121(.a(new_n207), .b(new_n209), .o1(new_n217));
  tech160nm_fixorc02aa1n03p5x5 g122(.a(\a[20] ), .b(\b[19] ), .out0(new_n218));
  nand02aa1d28x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  oai022aa1n03x5               g125(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n221));
  nona22aa1n02x5               g126(.a(new_n217), .b(new_n220), .c(new_n221), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n218), .c(new_n216), .d(new_n217), .o1(\s[20] ));
  nano22aa1d15x5               g128(.a(new_n203), .b(new_n209), .c(new_n218), .out0(new_n224));
  inv000aa1d42x5               g129(.a(\a[20] ), .o1(new_n225));
  nanb02aa1n12x5               g130(.a(\b[19] ), .b(new_n225), .out0(new_n226));
  oai112aa1n06x5               g131(.a(new_n226), .b(new_n219), .c(new_n215), .d(new_n214), .o1(new_n227));
  nand03aa1n02x5               g132(.a(new_n204), .b(new_n216), .c(new_n200), .o1(new_n228));
  nand42aa1n02x5               g133(.a(new_n221), .b(new_n219), .o1(new_n229));
  oai012aa1n06x5               g134(.a(new_n229), .b(new_n228), .c(new_n227), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[21] ), .b(\b[20] ), .out0(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n230), .c(new_n194), .d(new_n224), .o1(new_n232));
  nano32aa1n02x4               g137(.a(new_n227), .b(new_n204), .c(new_n216), .d(new_n200), .out0(new_n233));
  nona22aa1n02x4               g138(.a(new_n229), .b(new_n233), .c(new_n231), .out0(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n194), .c(new_n224), .o1(new_n235));
  norb02aa1n03x4               g140(.a(new_n232), .b(new_n235), .out0(\s[21] ));
  inv040aa1d30x5               g141(.a(\a[21] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(\b[20] ), .b(new_n237), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  and002aa1n02x5               g144(.a(\b[21] ), .b(\a[22] ), .o(new_n240));
  oai022aa1n02x5               g145(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n241));
  nona22aa1n02x5               g146(.a(new_n232), .b(new_n240), .c(new_n241), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n242), .b(new_n239), .c(new_n238), .d(new_n232), .o1(\s[22] ));
  inv040aa1d32x5               g148(.a(\a[22] ), .o1(new_n244));
  xroi22aa1d06x4               g149(.a(new_n237), .b(\b[20] ), .c(new_n244), .d(\b[21] ), .out0(new_n245));
  nano32aa1n02x4               g150(.a(new_n203), .b(new_n245), .c(new_n209), .d(new_n218), .out0(new_n246));
  oaoi03aa1n02x5               g151(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n247));
  tech160nm_fiao0012aa1n02p5x5 g152(.a(new_n247), .b(new_n230), .c(new_n245), .o(new_n248));
  xorc02aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n194), .d(new_n246), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n247), .c(new_n230), .d(new_n245), .o1(new_n251));
  aobi12aa1n02x5               g156(.a(new_n251), .b(new_n194), .c(new_n246), .out0(new_n252));
  norb02aa1n03x4               g157(.a(new_n250), .b(new_n252), .out0(\s[23] ));
  norp02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .out0(new_n256));
  and002aa1n02x5               g161(.a(\b[23] ), .b(\a[24] ), .o(new_n257));
  oai022aa1n02x5               g162(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n258));
  nona22aa1n02x5               g163(.a(new_n250), .b(new_n257), .c(new_n258), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n256), .c(new_n255), .d(new_n250), .o1(\s[24] ));
  inv000aa1d42x5               g165(.a(\a[23] ), .o1(new_n261));
  inv040aa1d32x5               g166(.a(\a[24] ), .o1(new_n262));
  xroi22aa1d06x4               g167(.a(new_n261), .b(\b[22] ), .c(new_n262), .d(\b[23] ), .out0(new_n263));
  nanp02aa1n03x5               g168(.a(new_n263), .b(new_n245), .o1(new_n264));
  nano23aa1d12x5               g169(.a(new_n264), .b(new_n203), .c(new_n209), .d(new_n218), .out0(new_n265));
  aoai13aa1n04x5               g170(.a(new_n263), .b(new_n247), .c(new_n230), .d(new_n245), .o1(new_n266));
  oaib12aa1n02x5               g171(.a(new_n258), .b(new_n262), .c(\b[23] ), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[25] ), .b(\b[24] ), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n268), .c(new_n194), .d(new_n265), .o1(new_n270));
  nanb02aa1n02x5               g175(.a(new_n269), .b(new_n267), .out0(new_n271));
  aoi122aa1n06x5               g176(.a(new_n271), .b(new_n248), .c(new_n263), .d(new_n194), .e(new_n265), .o1(new_n272));
  norb02aa1n02x5               g177(.a(new_n270), .b(new_n272), .out0(\s[25] ));
  norp02aa1n02x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  tech160nm_fixorc02aa1n02p5x5 g180(.a(\a[26] ), .b(\b[25] ), .out0(new_n276));
  and002aa1n02x5               g181(.a(\b[25] ), .b(\a[26] ), .o(new_n277));
  oai022aa1n02x5               g182(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n278));
  nona22aa1n02x5               g183(.a(new_n270), .b(new_n277), .c(new_n278), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n275), .d(new_n270), .o1(\s[26] ));
  and002aa1n06x5               g185(.a(new_n276), .b(new_n269), .o(new_n281));
  nanb03aa1n09x5               g186(.a(new_n264), .b(new_n224), .c(new_n281), .out0(new_n282));
  aoi012aa1n12x5               g187(.a(new_n282), .b(new_n186), .c(new_n193), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n281), .o1(new_n284));
  aob012aa1n02x5               g189(.a(new_n278), .b(\b[25] ), .c(\a[26] ), .out0(new_n285));
  aoai13aa1n09x5               g190(.a(new_n285), .b(new_n284), .c(new_n266), .d(new_n267), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  nanb02aa1n02x5               g192(.a(new_n287), .b(new_n285), .out0(new_n288));
  aoi112aa1n03x4               g193(.a(new_n283), .b(new_n288), .c(new_n268), .d(new_n281), .o1(new_n289));
  oaoi13aa1n02x7               g194(.a(new_n289), .b(new_n287), .c(new_n283), .d(new_n286), .o1(\s[27] ));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  tech160nm_fioai012aa1n04x5   g197(.a(new_n287), .b(new_n286), .c(new_n283), .o1(new_n293));
  xorc02aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .out0(new_n294));
  oai022aa1d24x5               g199(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(\a[28] ), .c(\b[27] ), .o1(new_n296));
  tech160nm_finand02aa1n03p5x5 g201(.a(new_n293), .b(new_n296), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n293), .d(new_n292), .o1(\s[28] ));
  xorc02aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .out0(new_n299));
  and002aa1n02x5               g204(.a(new_n294), .b(new_n287), .o(new_n300));
  oaih12aa1n02x5               g205(.a(new_n300), .b(new_n286), .c(new_n283), .o1(new_n301));
  inv000aa1d42x5               g206(.a(\b[27] ), .o1(new_n302));
  oaib12aa1n09x5               g207(.a(new_n295), .b(new_n302), .c(\a[28] ), .out0(new_n303));
  nanp03aa1n03x5               g208(.a(new_n301), .b(new_n303), .c(new_n299), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n303), .o1(new_n305));
  oaoi13aa1n03x5               g210(.a(new_n305), .b(new_n300), .c(new_n286), .d(new_n283), .o1(new_n306));
  oaih12aa1n02x5               g211(.a(new_n304), .b(new_n306), .c(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g212(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g213(.a(new_n287), .b(new_n299), .c(new_n294), .o(new_n309));
  tech160nm_fioaoi03aa1n03p5x5 g214(.a(\a[29] ), .b(\b[28] ), .c(new_n303), .o1(new_n310));
  oaoi13aa1n03x5               g215(.a(new_n310), .b(new_n309), .c(new_n286), .d(new_n283), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .out0(new_n312));
  oaih12aa1n02x5               g217(.a(new_n309), .b(new_n286), .c(new_n283), .o1(new_n313));
  norb02aa1n03x5               g218(.a(new_n312), .b(new_n310), .out0(new_n314));
  tech160nm_finand02aa1n05x5   g219(.a(new_n313), .b(new_n314), .o1(new_n315));
  oaih12aa1n02x5               g220(.a(new_n315), .b(new_n311), .c(new_n312), .o1(\s[30] ));
  xorc02aa1n02x5               g221(.a(\a[31] ), .b(\b[30] ), .out0(new_n317));
  and003aa1n02x5               g222(.a(new_n300), .b(new_n312), .c(new_n299), .o(new_n318));
  aoi012aa1n02x5               g223(.a(new_n314), .b(\a[30] ), .c(\b[29] ), .o1(new_n319));
  oaoi13aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n286), .d(new_n283), .o1(new_n320));
  oaih12aa1n02x5               g225(.a(new_n318), .b(new_n286), .c(new_n283), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n317), .b(new_n319), .out0(new_n322));
  tech160nm_finand02aa1n03p5x5 g227(.a(new_n321), .b(new_n322), .o1(new_n323));
  oaih12aa1n02x5               g228(.a(new_n323), .b(new_n320), .c(new_n317), .o1(\s[31] ));
  nano22aa1n02x4               g229(.a(new_n97), .b(new_n98), .c(new_n99), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n104), .b(new_n103), .out0(new_n326));
  nona22aa1n02x4               g231(.a(new_n104), .b(new_n103), .c(new_n97), .out0(new_n327));
  oai022aa1n02x5               g232(.a(new_n327), .b(new_n325), .c(new_n326), .d(new_n100), .o1(\s[3] ));
  nanb02aa1n02x5               g233(.a(new_n101), .b(new_n102), .out0(new_n329));
  oai012aa1n02x5               g234(.a(new_n104), .b(new_n327), .c(new_n325), .o1(new_n330));
  aboi22aa1n03x5               g235(.a(new_n101), .b(new_n107), .c(new_n330), .d(new_n329), .out0(\s[4] ));
  nona22aa1n02x4               g236(.a(new_n326), .b(new_n100), .c(new_n329), .out0(new_n332));
  norb02aa1n02x5               g237(.a(new_n116), .b(new_n115), .out0(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n333), .b(new_n332), .c(new_n106), .out0(\s[5] ));
  norb02aa1n02x5               g239(.a(new_n114), .b(new_n113), .out0(new_n335));
  aoai13aa1n02x5               g240(.a(new_n335), .b(new_n115), .c(new_n107), .d(new_n116), .o1(new_n336));
  aoi112aa1n02x5               g241(.a(new_n115), .b(new_n335), .c(new_n107), .d(new_n333), .o1(new_n337));
  norb02aa1n02x5               g242(.a(new_n336), .b(new_n337), .out0(\s[6] ));
  inv000aa1d42x5               g243(.a(new_n113), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n111), .b(new_n336), .c(new_n339), .out0(\s[7] ));
  aob012aa1n02x5               g245(.a(new_n111), .b(new_n336), .c(new_n339), .out0(new_n341));
  xobna2aa1n03x5               g246(.a(new_n108), .b(new_n341), .c(new_n123), .out0(\s[8] ));
  nona22aa1n02x4               g247(.a(new_n119), .b(new_n122), .c(new_n124), .out0(new_n343));
  norb02aa1n02x5               g248(.a(new_n128), .b(new_n125), .out0(new_n344));
  norp03aa1n02x5               g249(.a(new_n122), .b(new_n124), .c(new_n344), .o1(new_n345));
  aoi022aa1n02x5               g250(.a(new_n343), .b(new_n344), .c(new_n119), .d(new_n345), .o1(\s[9] ));
endmodule


