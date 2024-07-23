// Benchmark "adder" written by ABC on Thu Jul 18 03:27:19 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n340, new_n341, new_n342, new_n344, new_n345,
    new_n348, new_n349, new_n351, new_n352, new_n354, new_n356, new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nona22aa1n09x5               g004(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n100));
  tech160nm_fioai012aa1n04x5   g005(.a(new_n98), .b(\b[3] ), .c(\a[4] ), .o1(new_n101));
  nand22aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\a[4] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[3] ), .o1(new_n104));
  oai022aa1n03x5               g009(.a(new_n103), .b(new_n104), .c(\a[3] ), .d(\b[2] ), .o1(new_n105));
  nona23aa1n09x5               g010(.a(new_n100), .b(new_n102), .c(new_n105), .d(new_n101), .out0(new_n106));
  nor002aa1n20x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  oaoi03aa1n12x5               g012(.a(new_n103), .b(new_n104), .c(new_n107), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .out0(new_n109));
  xorc02aa1n12x5               g014(.a(\a[5] ), .b(\b[4] ), .out0(new_n110));
  xorc02aa1n12x5               g015(.a(\a[8] ), .b(\b[7] ), .out0(new_n111));
  nor042aa1d18x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n08x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanb02aa1n03x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  nona23aa1n09x5               g019(.a(new_n110), .b(new_n111), .c(new_n109), .d(new_n114), .out0(new_n115));
  tech160nm_finand02aa1n03p5x5 g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nano22aa1n03x7               g021(.a(new_n112), .b(new_n116), .c(new_n113), .out0(new_n117));
  oaih22aa1n04x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  inv000aa1n03x5               g023(.a(new_n112), .o1(new_n119));
  oaoi03aa1n03x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  aoi013aa1n06x4               g025(.a(new_n120), .b(new_n117), .c(new_n111), .d(new_n118), .o1(new_n121));
  aoai13aa1n12x5               g026(.a(new_n121), .b(new_n115), .c(new_n106), .d(new_n108), .o1(new_n122));
  nor042aa1n06x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  nor042aa1n06x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n08x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n06x4               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  aoi112aa1n02x5               g032(.a(new_n123), .b(new_n127), .c(new_n122), .d(new_n124), .o1(new_n128));
  aoai13aa1n06x5               g033(.a(new_n127), .b(new_n123), .c(new_n122), .d(new_n124), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(\s[10] ));
  oai012aa1n02x5               g035(.a(new_n126), .b(new_n125), .c(new_n123), .o1(new_n131));
  nor042aa1d18x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n129), .c(new_n131), .out0(\s[11] ));
  nanp02aa1n03x5               g040(.a(new_n129), .b(new_n131), .o1(new_n136));
  nanp02aa1n03x5               g041(.a(new_n136), .b(new_n134), .o1(new_n137));
  nor042aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoib12aa1n02x5               g045(.a(new_n132), .b(new_n139), .c(new_n138), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n132), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n134), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n142), .b(new_n143), .c(new_n129), .d(new_n131), .o1(new_n144));
  aoi022aa1n03x5               g049(.a(new_n144), .b(new_n140), .c(new_n137), .d(new_n141), .o1(\s[12] ));
  nanp02aa1n02x5               g050(.a(new_n106), .b(new_n108), .o1(new_n146));
  xorc02aa1n02x5               g051(.a(\a[6] ), .b(\b[5] ), .out0(new_n147));
  norb02aa1n03x5               g052(.a(new_n113), .b(new_n112), .out0(new_n148));
  nanp02aa1n02x5               g053(.a(new_n111), .b(new_n148), .o1(new_n149));
  nano22aa1n03x7               g054(.a(new_n149), .b(new_n147), .c(new_n110), .out0(new_n150));
  nanp03aa1n02x5               g055(.a(new_n117), .b(new_n111), .c(new_n118), .o1(new_n151));
  nanb02aa1n02x5               g056(.a(new_n120), .b(new_n151), .out0(new_n152));
  nano23aa1n09x5               g057(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n133), .out0(new_n153));
  nand23aa1d12x5               g058(.a(new_n153), .b(new_n124), .c(new_n127), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n152), .c(new_n146), .d(new_n150), .o1(new_n156));
  nano22aa1n09x5               g061(.a(new_n138), .b(new_n133), .c(new_n139), .out0(new_n157));
  oai012aa1n02x5               g062(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .o1(new_n158));
  oab012aa1n04x5               g063(.a(new_n158), .b(new_n123), .c(new_n125), .out0(new_n159));
  aoi012aa1n06x5               g064(.a(new_n138), .b(new_n132), .c(new_n139), .o1(new_n160));
  aob012aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n157), .out0(new_n161));
  nanb02aa1n02x5               g066(.a(new_n161), .b(new_n156), .out0(new_n162));
  norp02aa1n09x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nand02aa1n06x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  inv040aa1n03x5               g070(.a(new_n160), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n166), .b(new_n165), .c(new_n159), .d(new_n157), .o1(new_n167));
  aoi022aa1n02x5               g072(.a(new_n162), .b(new_n165), .c(new_n156), .d(new_n167), .o1(\s[13] ));
  orn002aa1n02x5               g073(.a(\a[13] ), .b(\b[12] ), .o(new_n169));
  aoai13aa1n03x5               g074(.a(new_n165), .b(new_n161), .c(new_n122), .d(new_n155), .o1(new_n170));
  nor042aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n16x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  xnbna2aa1n03x5               g078(.a(new_n173), .b(new_n170), .c(new_n169), .out0(\s[14] ));
  nano23aa1d12x5               g079(.a(new_n163), .b(new_n171), .c(new_n172), .d(new_n164), .out0(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n161), .c(new_n122), .d(new_n155), .o1(new_n176));
  aoi012aa1d18x5               g081(.a(new_n171), .b(new_n163), .c(new_n172), .o1(new_n177));
  xorc02aa1n12x5               g082(.a(\a[15] ), .b(\b[14] ), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n176), .c(new_n177), .out0(\s[15] ));
  inv000aa1d42x5               g084(.a(new_n177), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n178), .b(new_n180), .c(new_n162), .d(new_n175), .o1(new_n181));
  tech160nm_fixorc02aa1n03p5x5 g086(.a(\a[16] ), .b(\b[15] ), .out0(new_n182));
  nor042aa1n06x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norp02aa1n02x5               g088(.a(new_n182), .b(new_n183), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n183), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n178), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n176), .d(new_n177), .o1(new_n187));
  aoi022aa1n03x5               g092(.a(new_n187), .b(new_n182), .c(new_n181), .d(new_n184), .o1(\s[16] ));
  nano32aa1d12x5               g093(.a(new_n154), .b(new_n182), .c(new_n175), .d(new_n178), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n152), .c(new_n146), .d(new_n150), .o1(new_n190));
  and002aa1n02x5               g095(.a(new_n182), .b(new_n178), .o(new_n191));
  aoai13aa1n04x5               g096(.a(new_n175), .b(new_n166), .c(new_n159), .d(new_n157), .o1(new_n192));
  aob012aa1n06x5               g097(.a(new_n191), .b(new_n192), .c(new_n177), .out0(new_n193));
  oao003aa1n02x5               g098(.a(\a[16] ), .b(\b[15] ), .c(new_n185), .carry(new_n194));
  nand23aa1n06x5               g099(.a(new_n190), .b(new_n193), .c(new_n194), .o1(new_n195));
  xorc02aa1n12x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  nano22aa1n02x4               g101(.a(new_n196), .b(new_n193), .c(new_n194), .out0(new_n197));
  aoi022aa1n02x5               g102(.a(new_n197), .b(new_n190), .c(new_n195), .d(new_n196), .o1(\s[17] ));
  nor002aa1d32x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n182), .b(new_n178), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n194), .b(new_n201), .c(new_n192), .d(new_n177), .o1(new_n202));
  aoai13aa1n03x5               g107(.a(new_n196), .b(new_n202), .c(new_n122), .d(new_n189), .o1(new_n203));
  nor002aa1n16x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand22aa1n09x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n06x4               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(new_n200), .out0(\s[18] ));
  and002aa1n02x5               g112(.a(new_n196), .b(new_n206), .o(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n202), .c(new_n122), .d(new_n189), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nor042aa1d18x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nand02aa1n06x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n12x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g121(.a(new_n214), .b(new_n210), .c(new_n195), .d(new_n208), .o1(new_n217));
  nor002aa1d32x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d16x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[18] ), .o1(new_n222));
  aboi22aa1n03x5               g127(.a(new_n218), .b(new_n219), .c(new_n221), .d(new_n222), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n212), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n214), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n224), .b(new_n225), .c(new_n209), .d(new_n211), .o1(new_n226));
  aoi022aa1n03x5               g131(.a(new_n226), .b(new_n220), .c(new_n217), .d(new_n223), .o1(\s[20] ));
  nano32aa1n03x7               g132(.a(new_n225), .b(new_n196), .c(new_n220), .d(new_n206), .out0(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n202), .c(new_n122), .d(new_n189), .o1(new_n229));
  nanb03aa1n06x5               g134(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n230));
  oai112aa1n06x5               g135(.a(new_n224), .b(new_n205), .c(new_n204), .d(new_n199), .o1(new_n231));
  aoi012aa1d18x5               g136(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n232));
  oaih12aa1n12x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  nanp02aa1n02x5               g139(.a(new_n229), .b(new_n234), .o1(new_n235));
  nor002aa1d32x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nanp02aa1n04x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n03x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  nano22aa1n03x5               g143(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n239));
  tech160nm_fioai012aa1n04x5   g144(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .o1(new_n240));
  oab012aa1n02x5               g145(.a(new_n240), .b(new_n199), .c(new_n204), .out0(new_n241));
  inv000aa1n02x5               g146(.a(new_n232), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n242), .b(new_n238), .c(new_n241), .d(new_n239), .o1(new_n243));
  aoi022aa1n02x5               g148(.a(new_n235), .b(new_n238), .c(new_n229), .d(new_n243), .o1(\s[21] ));
  aoai13aa1n03x5               g149(.a(new_n238), .b(new_n233), .c(new_n195), .d(new_n228), .o1(new_n245));
  nor042aa1n06x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  nand02aa1n10x5               g151(.a(\b[21] ), .b(\a[22] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  aoib12aa1n02x5               g153(.a(new_n236), .b(new_n247), .c(new_n246), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n236), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n238), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n250), .b(new_n251), .c(new_n229), .d(new_n234), .o1(new_n252));
  aoi022aa1n03x5               g157(.a(new_n252), .b(new_n248), .c(new_n245), .d(new_n249), .o1(\s[22] ));
  inv000aa1n02x5               g158(.a(new_n228), .o1(new_n254));
  nano22aa1n02x5               g159(.a(new_n254), .b(new_n238), .c(new_n248), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n202), .c(new_n122), .d(new_n189), .o1(new_n256));
  nano23aa1d15x5               g161(.a(new_n236), .b(new_n246), .c(new_n247), .d(new_n237), .out0(new_n257));
  aoi012aa1d18x5               g162(.a(new_n246), .b(new_n236), .c(new_n247), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  tech160nm_fiaoi012aa1n03p5x5 g164(.a(new_n259), .b(new_n233), .c(new_n257), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n256), .b(new_n260), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n262), .b(new_n259), .c(new_n233), .d(new_n257), .o1(new_n263));
  aoi022aa1n02x5               g168(.a(new_n261), .b(new_n262), .c(new_n256), .d(new_n263), .o1(\s[23] ));
  inv000aa1n02x5               g169(.a(new_n260), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n262), .b(new_n265), .c(new_n195), .d(new_n255), .o1(new_n266));
  tech160nm_fixorc02aa1n02p5x5 g171(.a(\a[24] ), .b(\b[23] ), .out0(new_n267));
  nor042aa1n06x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n267), .b(new_n268), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n262), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n270), .b(new_n271), .c(new_n256), .d(new_n260), .o1(new_n272));
  aoi022aa1n03x5               g177(.a(new_n272), .b(new_n267), .c(new_n266), .d(new_n269), .o1(\s[24] ));
  and002aa1n12x5               g178(.a(new_n267), .b(new_n262), .o(new_n274));
  nano22aa1n02x5               g179(.a(new_n254), .b(new_n274), .c(new_n257), .out0(new_n275));
  aoai13aa1n04x5               g180(.a(new_n275), .b(new_n202), .c(new_n122), .d(new_n189), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n257), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n277));
  inv000aa1n06x5               g182(.a(new_n274), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[24] ), .b(\b[23] ), .c(new_n270), .carry(new_n279));
  aoai13aa1n12x5               g184(.a(new_n279), .b(new_n278), .c(new_n277), .d(new_n258), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(new_n276), .b(new_n281), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n274), .b(new_n259), .c(new_n233), .d(new_n257), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n283), .o1(new_n285));
  and003aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n279), .o(new_n286));
  aoi022aa1n02x5               g191(.a(new_n282), .b(new_n283), .c(new_n276), .d(new_n286), .o1(\s[25] ));
  aoai13aa1n03x5               g192(.a(new_n283), .b(new_n280), .c(new_n195), .d(new_n275), .o1(new_n288));
  tech160nm_fixorc02aa1n02p5x5 g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  nor042aa1n03x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n289), .b(new_n290), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n290), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n285), .c(new_n276), .d(new_n281), .o1(new_n293));
  aoi022aa1n03x5               g198(.a(new_n293), .b(new_n289), .c(new_n288), .d(new_n291), .o1(\s[26] ));
  and002aa1n12x5               g199(.a(new_n289), .b(new_n283), .o(new_n295));
  nano32aa1n03x7               g200(.a(new_n254), .b(new_n295), .c(new_n257), .d(new_n274), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n202), .c(new_n122), .d(new_n189), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n292), .carry(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoi012aa1d18x5               g204(.a(new_n299), .b(new_n280), .c(new_n295), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n297), .b(new_n300), .o1(new_n301));
  xorc02aa1n12x5               g206(.a(\a[27] ), .b(\b[26] ), .out0(new_n302));
  aoi112aa1n02x5               g207(.a(new_n302), .b(new_n299), .c(new_n280), .d(new_n295), .o1(new_n303));
  aoi022aa1n02x5               g208(.a(new_n301), .b(new_n302), .c(new_n297), .d(new_n303), .o1(\s[27] ));
  inv000aa1d42x5               g209(.a(new_n295), .o1(new_n305));
  aoai13aa1n04x5               g210(.a(new_n298), .b(new_n305), .c(new_n284), .d(new_n279), .o1(new_n306));
  aoai13aa1n02x5               g211(.a(new_n302), .b(new_n306), .c(new_n195), .d(new_n296), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(\b[26] ), .b(\a[27] ), .o1(new_n309));
  norp02aa1n02x5               g214(.a(new_n308), .b(new_n309), .o1(new_n310));
  inv000aa1n03x5               g215(.a(new_n309), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n302), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n311), .b(new_n312), .c(new_n297), .d(new_n300), .o1(new_n313));
  aoi022aa1n03x5               g218(.a(new_n313), .b(new_n308), .c(new_n307), .d(new_n310), .o1(\s[28] ));
  and002aa1n02x5               g219(.a(new_n308), .b(new_n302), .o(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n306), .c(new_n195), .d(new_n296), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n311), .carry(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n297), .d(new_n300), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g227(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g228(.a(new_n312), .b(new_n308), .c(new_n320), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n306), .c(new_n195), .d(new_n296), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n324), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .carry(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n326), .c(new_n297), .d(new_n300), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .out0(new_n329));
  norb02aa1n02x5               g234(.a(new_n327), .b(new_n329), .out0(new_n330));
  aoi022aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n325), .d(new_n330), .o1(\s[30] ));
  nano32aa1n03x7               g236(.a(new_n312), .b(new_n329), .c(new_n308), .d(new_n320), .out0(new_n332));
  aoai13aa1n02x5               g237(.a(new_n332), .b(new_n306), .c(new_n195), .d(new_n296), .o1(new_n333));
  xorc02aa1n02x5               g238(.a(\a[31] ), .b(\b[30] ), .out0(new_n334));
  oao003aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .c(new_n327), .carry(new_n335));
  norb02aa1n02x5               g240(.a(new_n335), .b(new_n334), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n332), .o1(new_n337));
  aoai13aa1n03x5               g242(.a(new_n335), .b(new_n337), .c(new_n297), .d(new_n300), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n338), .b(new_n334), .c(new_n333), .d(new_n336), .o1(\s[31] ));
  nano32aa1n02x4               g244(.a(new_n107), .b(new_n100), .c(new_n102), .d(new_n98), .out0(new_n340));
  inv000aa1d42x5               g245(.a(new_n107), .o1(new_n341));
  aoi022aa1n02x5               g246(.a(new_n100), .b(new_n98), .c(new_n341), .d(new_n102), .o1(new_n342));
  norp02aa1n02x5               g247(.a(new_n340), .b(new_n342), .o1(\s[3] ));
  inv000aa1n03x5               g248(.a(new_n340), .o1(new_n344));
  xorc02aa1n02x5               g249(.a(\a[4] ), .b(\b[3] ), .out0(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n345), .b(new_n344), .c(new_n341), .out0(\s[4] ));
  xnbna2aa1n03x5               g251(.a(new_n110), .b(new_n106), .c(new_n108), .out0(\s[5] ));
  orn002aa1n02x5               g252(.a(\a[5] ), .b(\b[4] ), .o(new_n348));
  nanp02aa1n02x5               g253(.a(new_n146), .b(new_n110), .o1(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n147), .b(new_n349), .c(new_n348), .out0(\s[6] ));
  nanp02aa1n02x5               g255(.a(new_n118), .b(new_n116), .o1(new_n351));
  nanp03aa1n02x5               g256(.a(new_n146), .b(new_n147), .c(new_n110), .o1(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n148), .b(new_n352), .c(new_n351), .out0(\s[7] ));
  aob012aa1n02x5               g258(.a(new_n148), .b(new_n352), .c(new_n351), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n111), .b(new_n354), .c(new_n119), .out0(\s[8] ));
  nanp02aa1n02x5               g260(.a(new_n146), .b(new_n150), .o1(new_n356));
  aoi113aa1n02x5               g261(.a(new_n124), .b(new_n120), .c(new_n117), .d(new_n111), .e(new_n118), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n122), .b(new_n124), .c(new_n356), .d(new_n357), .o1(\s[9] ));
endmodule


