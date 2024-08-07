// Benchmark "adder" written by ABC on Thu Jul 18 05:31:25 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n323, new_n325,
    new_n327, new_n329, new_n330, new_n331, new_n333, new_n334, new_n335;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv030aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand02aa1d06x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  nand22aa1n03x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  aoi012aa1n12x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n09x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nor022aa1n16x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  tech160nm_finand02aa1n03p5x5 g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nona23aa1n09x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  aoi012aa1n06x5               g014(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n110));
  oai012aa1n12x5               g015(.a(new_n110), .b(new_n109), .c(new_n104), .o1(new_n111));
  nor022aa1n08x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand22aa1n09x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  norp02aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand22aa1n06x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nanb02aa1n06x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  xorc02aa1n12x5               g024(.a(\a[5] ), .b(\b[4] ), .out0(new_n120));
  norb03aa1n03x5               g025(.a(new_n120), .b(new_n116), .c(new_n119), .out0(new_n121));
  inv000aa1d42x5               g026(.a(\a[5] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\b[4] ), .o1(new_n123));
  aoi013aa1n06x4               g028(.a(new_n117), .b(new_n118), .c(new_n122), .d(new_n123), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n125));
  oaih12aa1n02x5               g030(.a(new_n125), .b(new_n116), .c(new_n124), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n100), .b(new_n126), .c(new_n121), .d(new_n111), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1d28x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanb02aa1d36x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  aoi012aa1n02x5               g037(.a(new_n130), .b(new_n127), .c(new_n99), .o1(new_n133));
  aoai13aa1n06x5               g038(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n130), .c(new_n127), .d(new_n99), .o1(new_n135));
  nor042aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1d28x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  norb02aa1n02x5               g043(.a(new_n134), .b(new_n138), .out0(new_n139));
  aboi22aa1n03x5               g044(.a(new_n133), .b(new_n139), .c(new_n135), .d(new_n138), .out0(\s[11] ));
  inv000aa1n02x5               g045(.a(new_n136), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(new_n135), .b(new_n138), .o1(new_n142));
  nor002aa1n16x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1d16x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  norb03aa1n02x5               g050(.a(new_n144), .b(new_n136), .c(new_n143), .out0(new_n146));
  nanp02aa1n02x5               g051(.a(new_n142), .b(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n142), .d(new_n141), .o1(\s[12] ));
  aoi012aa1n02x5               g053(.a(new_n126), .b(new_n121), .c(new_n111), .o1(new_n149));
  nano23aa1n06x5               g054(.a(new_n136), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n99), .b(new_n100), .o1(new_n151));
  nona22aa1n09x5               g056(.a(new_n150), .b(new_n130), .c(new_n151), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n137), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n143), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n153), .c(new_n134), .d(new_n141), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n144), .o1(new_n156));
  tech160nm_fioai012aa1n05x5   g061(.a(new_n156), .b(new_n149), .c(new_n152), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  tech160nm_fixnrc02aa1n05x5   g063(.a(\b[12] ), .b(\a[13] ), .out0(new_n159));
  nanb02aa1n02x5               g064(.a(new_n159), .b(new_n157), .out0(new_n160));
  nor042aa1d18x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  tech160nm_fixnrc02aa1n05x5   g066(.a(\b[13] ), .b(\a[14] ), .out0(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  inv000aa1d42x5               g068(.a(\a[14] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[13] ), .o1(new_n165));
  nor002aa1n03x5               g070(.a(new_n162), .b(new_n159), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n157), .b(new_n166), .o1(new_n167));
  oaoi03aa1n12x5               g072(.a(new_n164), .b(new_n165), .c(new_n161), .o1(new_n168));
  aoi022aa1n02x5               g073(.a(new_n167), .b(new_n168), .c(new_n165), .d(new_n164), .o1(new_n169));
  aoi012aa1n02x5               g074(.a(new_n169), .b(new_n160), .c(new_n163), .o1(\s[14] ));
  nor002aa1d32x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n06x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n167), .c(new_n168), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n171), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n168), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n173), .b(new_n176), .c(new_n157), .d(new_n166), .o1(new_n177));
  nor022aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1n06x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  norb03aa1n02x5               g085(.a(new_n179), .b(new_n171), .c(new_n178), .out0(new_n181));
  nanp02aa1n02x5               g086(.a(new_n177), .b(new_n181), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n180), .c(new_n177), .d(new_n175), .o1(\s[16] ));
  nano23aa1n03x7               g088(.a(new_n171), .b(new_n178), .c(new_n179), .d(new_n172), .out0(new_n184));
  nano22aa1n06x5               g089(.a(new_n152), .b(new_n166), .c(new_n184), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n126), .c(new_n111), .d(new_n121), .o1(new_n186));
  nona23aa1n09x5               g091(.a(new_n179), .b(new_n172), .c(new_n171), .d(new_n178), .out0(new_n187));
  nor043aa1n03x5               g092(.a(new_n187), .b(new_n162), .c(new_n159), .o1(new_n188));
  oai012aa1n02x5               g093(.a(new_n179), .b(new_n178), .c(new_n171), .o1(new_n189));
  tech160nm_fioai012aa1n04x5   g094(.a(new_n189), .b(new_n187), .c(new_n168), .o1(new_n190));
  aoi013aa1n09x5               g095(.a(new_n190), .b(new_n188), .c(new_n155), .d(new_n144), .o1(new_n191));
  nanp02aa1n09x5               g096(.a(new_n186), .b(new_n191), .o1(new_n192));
  xorc02aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  aoai13aa1n02x5               g098(.a(new_n184), .b(new_n176), .c(new_n157), .d(new_n166), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n189), .b(new_n193), .out0(new_n195));
  aoi022aa1n02x5               g100(.a(new_n194), .b(new_n195), .c(new_n193), .d(new_n192), .o1(\s[17] ));
  norp02aa1n12x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nand02aa1d28x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  obai22aa1n02x7               g103(.a(new_n198), .b(new_n197), .c(\a[17] ), .d(\b[16] ), .out0(new_n199));
  aoi012aa1n02x5               g104(.a(new_n199), .b(new_n192), .c(new_n193), .o1(new_n200));
  inv040aa1n15x5               g105(.a(\a[17] ), .o1(new_n201));
  inv040aa1d32x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d06x4               g107(.a(new_n201), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  nand22aa1n03x5               g108(.a(new_n192), .b(new_n203), .o1(new_n204));
  inv000aa1n36x5               g109(.a(\b[16] ), .o1(new_n205));
  aoi013aa1n09x5               g110(.a(new_n197), .b(new_n198), .c(new_n201), .d(new_n205), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n197), .b(new_n204), .c(new_n206), .o1(new_n207));
  norp02aa1n02x5               g112(.a(new_n207), .b(new_n200), .o1(\s[18] ));
  nor002aa1d32x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nand02aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g118(.a(new_n209), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n206), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n211), .b(new_n215), .c(new_n192), .d(new_n203), .o1(new_n216));
  nor022aa1n16x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand02aa1n06x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  norb03aa1n02x5               g124(.a(new_n218), .b(new_n209), .c(new_n217), .out0(new_n220));
  nand42aa1n02x5               g125(.a(new_n216), .b(new_n220), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n216), .d(new_n214), .o1(\s[20] ));
  nona23aa1d18x5               g127(.a(new_n218), .b(new_n210), .c(new_n209), .d(new_n217), .out0(new_n223));
  inv000aa1n04x5               g128(.a(new_n223), .o1(new_n224));
  nand22aa1n12x5               g129(.a(new_n203), .b(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  oai012aa1n02x5               g131(.a(new_n218), .b(new_n217), .c(new_n209), .o1(new_n227));
  oai012aa1n12x5               g132(.a(new_n227), .b(new_n223), .c(new_n206), .o1(new_n228));
  xorc02aa1n06x5               g133(.a(\a[21] ), .b(\b[20] ), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n228), .c(new_n192), .d(new_n226), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n228), .c(new_n192), .d(new_n226), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(\s[21] ));
  inv000aa1d42x5               g137(.a(\a[21] ), .o1(new_n233));
  nanb02aa1n02x5               g138(.a(\b[20] ), .b(new_n233), .out0(new_n234));
  tech160nm_fixorc02aa1n03p5x5 g139(.a(\a[22] ), .b(\b[21] ), .out0(new_n235));
  nanp02aa1n02x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  oai022aa1n02x5               g141(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n236), .b(new_n237), .out0(new_n238));
  nanp02aa1n03x5               g143(.a(new_n230), .b(new_n238), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n235), .c(new_n230), .d(new_n234), .o1(\s[22] ));
  and002aa1n02x5               g145(.a(new_n235), .b(new_n229), .o(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n225), .out0(new_n242));
  aoi022aa1n06x5               g147(.a(new_n228), .b(new_n241), .c(new_n236), .d(new_n237), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n192), .d(new_n242), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n244), .c(new_n192), .d(new_n242), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n246), .b(new_n247), .out0(\s[23] ));
  norp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  and002aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .o(new_n252));
  oai022aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n253));
  nona22aa1n03x5               g158(.a(new_n246), .b(new_n252), .c(new_n253), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n251), .c(new_n246), .d(new_n250), .o1(\s[24] ));
  inv000aa1d42x5               g160(.a(\a[23] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(\a[24] ), .o1(new_n257));
  xroi22aa1d06x4               g162(.a(new_n256), .b(\b[22] ), .c(new_n257), .d(\b[23] ), .out0(new_n258));
  nano32aa1d12x5               g163(.a(new_n225), .b(new_n258), .c(new_n229), .d(new_n235), .out0(new_n259));
  oaoi03aa1n02x5               g164(.a(\a[22] ), .b(\b[21] ), .c(new_n234), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n258), .b(new_n260), .c(new_n228), .d(new_n241), .o1(new_n261));
  oaib12aa1n02x5               g166(.a(new_n253), .b(new_n257), .c(\b[23] ), .out0(new_n262));
  nanp02aa1n02x5               g167(.a(new_n261), .b(new_n262), .o1(new_n263));
  xorc02aa1n06x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n192), .d(new_n259), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n263), .c(new_n192), .d(new_n259), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  nanp02aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o1(new_n271));
  oai022aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n272));
  norb02aa1n02x5               g177(.a(new_n271), .b(new_n272), .out0(new_n273));
  nanp02aa1n03x5               g178(.a(new_n265), .b(new_n273), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n270), .c(new_n265), .d(new_n269), .o1(\s[26] ));
  and002aa1n06x5               g180(.a(new_n270), .b(new_n264), .o(new_n276));
  nand22aa1n06x5               g181(.a(new_n259), .b(new_n276), .o1(new_n277));
  aoi012aa1d24x5               g182(.a(new_n277), .b(new_n186), .c(new_n191), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n276), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n272), .b(new_n271), .o1(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n280), .c(new_n261), .d(new_n262), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  oai012aa1n06x5               g188(.a(new_n283), .b(new_n282), .c(new_n278), .o1(new_n284));
  aoi122aa1n02x5               g189(.a(new_n283), .b(new_n271), .c(new_n272), .d(new_n263), .e(new_n276), .o1(new_n285));
  aobi12aa1n03x7               g190(.a(new_n284), .b(new_n285), .c(new_n279), .out0(\s[27] ));
  inv000aa1d42x5               g191(.a(\a[27] ), .o1(new_n287));
  nanb02aa1n02x5               g192(.a(\b[26] ), .b(new_n287), .out0(new_n288));
  xorc02aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .out0(new_n289));
  oai022aa1n02x5               g194(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n290));
  aoi012aa1n02x5               g195(.a(new_n290), .b(\a[28] ), .c(\b[27] ), .o1(new_n291));
  tech160nm_finand02aa1n03p5x5 g196(.a(new_n284), .b(new_n291), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n284), .d(new_n288), .o1(\s[28] ));
  xorc02aa1n12x5               g198(.a(\a[29] ), .b(\b[28] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(\b[27] ), .o1(new_n295));
  xroi22aa1d04x5               g200(.a(new_n295), .b(\a[28] ), .c(new_n287), .d(\b[26] ), .out0(new_n296));
  oaih12aa1n02x5               g201(.a(new_n296), .b(new_n282), .c(new_n278), .o1(new_n297));
  oaoi03aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n294), .o1(new_n299));
  nona22aa1n02x5               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  oaoi13aa1n03x5               g205(.a(new_n298), .b(new_n296), .c(new_n282), .d(new_n278), .o1(new_n301));
  tech160nm_fioai012aa1n02p5x5 g206(.a(new_n300), .b(new_n301), .c(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g207(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g208(.a(new_n299), .b(new_n283), .c(new_n289), .out0(new_n304));
  aob012aa1n02x5               g209(.a(new_n298), .b(\b[28] ), .c(\a[29] ), .out0(new_n305));
  oai012aa1n02x5               g210(.a(new_n305), .b(\b[28] ), .c(\a[29] ), .o1(new_n306));
  oaoi13aa1n03x5               g211(.a(new_n306), .b(new_n304), .c(new_n282), .d(new_n278), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n304), .b(new_n282), .c(new_n278), .o1(new_n309));
  norb02aa1n02x5               g214(.a(new_n308), .b(new_n306), .out0(new_n310));
  nanp02aa1n02x5               g215(.a(new_n309), .b(new_n310), .o1(new_n311));
  oaih12aa1n02x5               g216(.a(new_n311), .b(new_n307), .c(new_n308), .o1(\s[30] ));
  xorc02aa1n02x5               g217(.a(\a[31] ), .b(\b[30] ), .out0(new_n313));
  nano32aa1n02x4               g218(.a(new_n299), .b(new_n308), .c(new_n283), .d(new_n289), .out0(new_n314));
  oaih12aa1n02x5               g219(.a(new_n314), .b(new_n282), .c(new_n278), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n310), .b(\a[30] ), .c(\b[29] ), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n313), .b(new_n316), .out0(new_n317));
  nand02aa1n04x5               g222(.a(new_n315), .b(new_n317), .o1(new_n318));
  oaoi13aa1n03x5               g223(.a(new_n316), .b(new_n314), .c(new_n282), .d(new_n278), .o1(new_n319));
  tech160nm_fioai012aa1n02p5x5 g224(.a(new_n318), .b(new_n319), .c(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g225(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nanb02aa1n02x5               g226(.a(new_n105), .b(new_n106), .out0(new_n322));
  oaib12aa1n02x5               g227(.a(new_n108), .b(new_n107), .c(new_n104), .out0(new_n323));
  aboi22aa1n03x5               g228(.a(new_n105), .b(new_n111), .c(new_n323), .d(new_n322), .out0(\s[4] ));
  orn002aa1n02x5               g229(.a(new_n109), .b(new_n104), .o(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n120), .b(new_n325), .c(new_n110), .out0(\s[5] ));
  oaoi03aa1n02x5               g231(.a(new_n122), .b(new_n123), .c(new_n111), .o1(new_n327));
  xnrb03aa1n02x5               g232(.a(new_n327), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g233(.a(new_n114), .b(new_n115), .out0(new_n329));
  inv000aa1d42x5               g234(.a(new_n329), .o1(new_n330));
  nanb03aa1n02x5               g235(.a(new_n119), .b(new_n111), .c(new_n120), .out0(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n330), .b(new_n331), .c(new_n124), .out0(\s[7] ));
  nanb02aa1n02x5               g237(.a(new_n112), .b(new_n113), .out0(new_n333));
  inv000aa1d42x5               g238(.a(new_n114), .o1(new_n334));
  aob012aa1n02x5               g239(.a(new_n330), .b(new_n331), .c(new_n124), .out0(new_n335));
  xobna2aa1n03x5               g240(.a(new_n333), .b(new_n335), .c(new_n334), .out0(\s[8] ));
  xnbna2aa1n03x5               g241(.a(new_n149), .b(new_n99), .c(new_n100), .out0(\s[9] ));
endmodule


