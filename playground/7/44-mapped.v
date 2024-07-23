// Benchmark "adder" written by ABC on Wed Jul 17 15:55:28 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n352, new_n354, new_n356, new_n358, new_n359, new_n360, new_n361,
    new_n363, new_n365, new_n367, new_n368;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1n03x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x4               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1n09x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv040aa1d28x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n12x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand02aa1n16x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  aob012aa1n12x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n107));
  nor042aa1n04x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand02aa1n08x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norb02aa1n06x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nor042aa1n04x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand02aa1n04x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  norb02aa1n09x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nanp03aa1d12x5               g018(.a(new_n107), .b(new_n110), .c(new_n113), .o1(new_n114));
  oai012aa1n12x5               g019(.a(new_n109), .b(new_n111), .c(new_n108), .o1(new_n115));
  nor002aa1n10x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand22aa1n09x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand22aa1n09x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nano23aa1n03x7               g024(.a(new_n119), .b(new_n116), .c(new_n117), .d(new_n118), .out0(new_n120));
  nor002aa1n20x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nand42aa1d28x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n121), .b(new_n122), .out0(new_n123));
  nand22aa1n12x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nor002aa1d32x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  nanb02aa1n03x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  nona22aa1n03x5               g031(.a(new_n120), .b(new_n123), .c(new_n126), .out0(new_n127));
  norb02aa1n12x5               g032(.a(new_n122), .b(new_n121), .out0(new_n128));
  norb02aa1n06x5               g033(.a(new_n124), .b(new_n125), .out0(new_n129));
  inv040aa1n02x5               g034(.a(new_n116), .o1(new_n130));
  nand42aa1n02x5               g035(.a(new_n119), .b(new_n117), .o1(new_n131));
  nand22aa1n03x5               g036(.a(new_n131), .b(new_n130), .o1(new_n132));
  inv040aa1n03x5               g037(.a(new_n125), .o1(new_n133));
  oaoi03aa1n12x5               g038(.a(\a[8] ), .b(\b[7] ), .c(new_n133), .o1(new_n134));
  aoi013aa1n06x4               g039(.a(new_n134), .b(new_n132), .c(new_n129), .d(new_n128), .o1(new_n135));
  aoai13aa1n12x5               g040(.a(new_n135), .b(new_n127), .c(new_n114), .d(new_n115), .o1(new_n136));
  tech160nm_fixorc02aa1n03p5x5 g041(.a(\a[9] ), .b(\b[8] ), .out0(new_n137));
  nanp02aa1n09x5               g042(.a(new_n136), .b(new_n137), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n99), .b(new_n138), .c(new_n101), .out0(\s[10] ));
  nor002aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nand42aa1n04x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n97), .o1(new_n143));
  inv000aa1n02x5               g048(.a(new_n98), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n143), .b(new_n144), .c(new_n138), .d(new_n101), .o1(new_n145));
  nand02aa1d04x5               g050(.a(new_n145), .b(new_n142), .o1(new_n146));
  aoi013aa1n02x4               g051(.a(new_n144), .b(new_n138), .c(new_n101), .d(new_n143), .o1(new_n147));
  oa0012aa1n03x5               g052(.a(new_n146), .b(new_n147), .c(new_n142), .o(\s[11] ));
  norp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanp02aa1n04x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  aoai13aa1n03x5               g056(.a(new_n151), .b(new_n140), .c(new_n145), .d(new_n141), .o1(new_n152));
  nona22aa1n02x4               g057(.a(new_n146), .b(new_n151), .c(new_n140), .out0(new_n153));
  nanp02aa1n03x5               g058(.a(new_n153), .b(new_n152), .o1(\s[12] ));
  nano32aa1n06x5               g059(.a(new_n151), .b(new_n137), .c(new_n142), .d(new_n99), .out0(new_n155));
  nanp02aa1n06x5               g060(.a(new_n136), .b(new_n155), .o1(new_n156));
  inv000aa1n02x5               g061(.a(new_n150), .o1(new_n157));
  oai012aa1n02x5               g062(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .o1(new_n158));
  oab012aa1n06x5               g063(.a(new_n140), .b(new_n97), .c(new_n100), .out0(new_n159));
  nona32aa1n09x5               g064(.a(new_n159), .b(new_n158), .c(new_n157), .d(new_n144), .out0(new_n160));
  tech160nm_fiaoi012aa1n05x5   g065(.a(new_n149), .b(new_n140), .c(new_n150), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n160), .b(new_n161), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  nanp02aa1n06x5               g068(.a(new_n156), .b(new_n163), .o1(new_n164));
  nor042aa1n04x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand42aa1n16x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  nano22aa1n02x4               g072(.a(new_n167), .b(new_n160), .c(new_n161), .out0(new_n168));
  aoi022aa1n02x5               g073(.a(new_n164), .b(new_n167), .c(new_n156), .d(new_n168), .o1(\s[13] ));
  tech160nm_fiaoi012aa1n05x5   g074(.a(new_n165), .b(new_n164), .c(new_n166), .o1(new_n170));
  xnrb03aa1n03x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n02x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n10x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nano23aa1n06x5               g078(.a(new_n165), .b(new_n172), .c(new_n173), .d(new_n166), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n162), .c(new_n136), .d(new_n155), .o1(new_n175));
  oa0012aa1n02x5               g080(.a(new_n173), .b(new_n172), .c(new_n165), .o(new_n176));
  nanb02aa1n03x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  norp02aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand22aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n06x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  oaih22aa1n04x5               g085(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n181));
  aboi22aa1n03x5               g086(.a(new_n178), .b(new_n179), .c(new_n181), .d(new_n173), .out0(new_n182));
  aoi022aa1n02x5               g087(.a(new_n177), .b(new_n180), .c(new_n175), .d(new_n182), .o1(\s[15] ));
  xorc02aa1n12x5               g088(.a(\a[16] ), .b(\b[15] ), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n178), .c(new_n177), .d(new_n179), .o1(new_n186));
  aoai13aa1n06x5               g091(.a(new_n180), .b(new_n176), .c(new_n164), .d(new_n174), .o1(new_n187));
  nona22aa1n02x4               g092(.a(new_n187), .b(new_n185), .c(new_n178), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n186), .b(new_n188), .o1(\s[16] ));
  nano23aa1n02x4               g094(.a(new_n140), .b(new_n149), .c(new_n150), .d(new_n141), .out0(new_n190));
  nand23aa1n04x5               g095(.a(new_n174), .b(new_n180), .c(new_n184), .o1(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n190), .c(new_n137), .d(new_n99), .out0(new_n192));
  nanp02aa1n09x5               g097(.a(new_n136), .b(new_n192), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n191), .b(new_n160), .c(new_n161), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[16] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[15] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[15] ), .b(\a[16] ), .o1(new_n198));
  nanp03aa1n02x5               g103(.a(new_n197), .b(new_n179), .c(new_n198), .o1(new_n199));
  oai112aa1n02x5               g104(.a(new_n181), .b(new_n173), .c(\b[14] ), .d(\a[15] ), .o1(new_n200));
  oaoi03aa1n02x5               g105(.a(new_n195), .b(new_n196), .c(new_n178), .o1(new_n201));
  oa0012aa1n06x5               g106(.a(new_n201), .b(new_n200), .c(new_n199), .o(new_n202));
  norb02aa1n06x4               g107(.a(new_n202), .b(new_n194), .out0(new_n203));
  nand02aa1d10x5               g108(.a(new_n193), .b(new_n203), .o1(new_n204));
  xorc02aa1n02x5               g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  orn002aa1n02x5               g110(.a(new_n200), .b(new_n199), .o(new_n206));
  nano23aa1n02x4               g111(.a(new_n194), .b(new_n205), .c(new_n206), .d(new_n201), .out0(new_n207));
  aoi022aa1n02x5               g112(.a(new_n204), .b(new_n205), .c(new_n193), .d(new_n207), .o1(\s[17] ));
  inv040aa1d32x5               g113(.a(\a[18] ), .o1(new_n209));
  nor042aa1n04x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  tech160nm_fiaoi012aa1n05x5   g115(.a(new_n210), .b(new_n204), .c(new_n205), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[17] ), .c(new_n209), .out0(\s[18] ));
  aoai13aa1n06x5               g117(.a(new_n202), .b(new_n191), .c(new_n160), .d(new_n161), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\a[17] ), .o1(new_n214));
  xroi22aa1d06x4               g119(.a(new_n214), .b(\b[16] ), .c(new_n209), .d(\b[17] ), .out0(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n213), .c(new_n136), .d(new_n192), .o1(new_n216));
  nand02aa1n06x5               g121(.a(\b[17] ), .b(\a[18] ), .o1(new_n217));
  nor042aa1n02x5               g122(.a(\b[17] ), .b(\a[18] ), .o1(new_n218));
  nor002aa1n04x5               g123(.a(new_n218), .b(new_n210), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n217), .b(new_n219), .out0(new_n220));
  inv000aa1n02x5               g125(.a(new_n220), .o1(new_n221));
  nor042aa1n04x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  nand42aa1n08x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  norb02aa1n09x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n216), .c(new_n221), .out0(\s[19] ));
  xnrc02aa1n02x5               g130(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g131(.a(new_n216), .b(new_n221), .o1(new_n227));
  nor042aa1n06x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nand42aa1n16x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  nanb02aa1n03x5               g134(.a(new_n228), .b(new_n229), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n222), .c(new_n227), .d(new_n223), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(new_n227), .b(new_n224), .o1(new_n232));
  nona22aa1n02x4               g137(.a(new_n232), .b(new_n230), .c(new_n222), .out0(new_n233));
  nanp02aa1n02x5               g138(.a(new_n233), .b(new_n231), .o1(\s[20] ));
  nanb03aa1d18x5               g139(.a(new_n230), .b(new_n215), .c(new_n224), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n213), .c(new_n136), .d(new_n192), .o1(new_n237));
  nanb03aa1n02x5               g142(.a(new_n228), .b(new_n229), .c(new_n223), .out0(new_n238));
  orn002aa1n02x5               g143(.a(\a[19] ), .b(\b[18] ), .o(new_n239));
  oai112aa1n02x5               g144(.a(new_n239), .b(new_n217), .c(new_n218), .d(new_n210), .o1(new_n240));
  aoi012aa1n06x5               g145(.a(new_n228), .b(new_n222), .c(new_n229), .o1(new_n241));
  oai012aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n238), .o1(new_n242));
  nanb02aa1n03x5               g147(.a(new_n242), .b(new_n237), .out0(new_n243));
  xorc02aa1n12x5               g148(.a(\a[21] ), .b(\b[20] ), .out0(new_n244));
  nano22aa1n12x5               g149(.a(new_n228), .b(new_n223), .c(new_n229), .out0(new_n245));
  oai012aa1d24x5               g150(.a(new_n217), .b(\b[18] ), .c(\a[19] ), .o1(new_n246));
  oab012aa1n02x5               g151(.a(new_n246), .b(new_n210), .c(new_n218), .out0(new_n247));
  inv040aa1n03x5               g152(.a(new_n241), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n248), .b(new_n244), .c(new_n247), .d(new_n245), .o1(new_n249));
  aoi022aa1n02x5               g154(.a(new_n243), .b(new_n244), .c(new_n237), .d(new_n249), .o1(\s[21] ));
  nor042aa1n03x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[21] ), .b(\a[22] ), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n251), .c(new_n243), .d(new_n244), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n244), .b(new_n242), .c(new_n204), .d(new_n236), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n251), .out0(new_n255));
  nanp02aa1n02x5               g160(.a(new_n253), .b(new_n255), .o1(\s[22] ));
  xnrc02aa1n02x5               g161(.a(\b[20] ), .b(\a[21] ), .out0(new_n257));
  nor042aa1n02x5               g162(.a(new_n252), .b(new_n257), .o1(new_n258));
  nano32aa1n02x4               g163(.a(new_n230), .b(new_n215), .c(new_n258), .d(new_n224), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n213), .c(new_n136), .d(new_n192), .o1(new_n260));
  nona22aa1n09x5               g165(.a(new_n245), .b(new_n219), .c(new_n246), .out0(new_n261));
  nanb02aa1n02x5               g166(.a(new_n252), .b(new_n244), .out0(new_n262));
  inv000aa1d42x5               g167(.a(\a[22] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(\b[21] ), .o1(new_n264));
  oaoi03aa1n12x5               g169(.a(new_n263), .b(new_n264), .c(new_n251), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n262), .c(new_n261), .d(new_n241), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  nanp02aa1n03x5               g172(.a(new_n260), .b(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[23] ), .b(\b[22] ), .out0(new_n269));
  inv040aa1n03x5               g174(.a(new_n265), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n242), .d(new_n258), .o1(new_n271));
  aoi022aa1n02x5               g176(.a(new_n268), .b(new_n269), .c(new_n260), .d(new_n271), .o1(\s[23] ));
  nor002aa1n02x5               g177(.a(\b[22] ), .b(\a[23] ), .o1(new_n273));
  xnrc02aa1n12x5               g178(.a(\b[23] ), .b(\a[24] ), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n273), .c(new_n268), .d(new_n269), .o1(new_n275));
  nand42aa1n02x5               g180(.a(new_n268), .b(new_n269), .o1(new_n276));
  nona22aa1n03x5               g181(.a(new_n276), .b(new_n274), .c(new_n273), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n277), .b(new_n275), .o1(\s[24] ));
  norb02aa1n06x5               g183(.a(new_n269), .b(new_n274), .out0(new_n279));
  nano22aa1n03x7               g184(.a(new_n235), .b(new_n258), .c(new_n279), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n213), .c(new_n136), .d(new_n192), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n258), .b(new_n248), .c(new_n247), .d(new_n245), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n279), .o1(new_n283));
  inv000aa1d42x5               g188(.a(\a[24] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\b[23] ), .o1(new_n285));
  oao003aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n273), .carry(new_n286));
  inv000aa1n02x5               g191(.a(new_n286), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n283), .c(new_n282), .d(new_n265), .o1(new_n288));
  nanb02aa1n06x5               g193(.a(new_n288), .b(new_n281), .out0(new_n289));
  xorc02aa1n12x5               g194(.a(\a[25] ), .b(\b[24] ), .out0(new_n290));
  aoi112aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n266), .d(new_n279), .o1(new_n291));
  aoi022aa1n02x5               g196(.a(new_n289), .b(new_n290), .c(new_n281), .d(new_n291), .o1(\s[25] ));
  norp02aa1n02x5               g197(.a(\b[24] ), .b(\a[25] ), .o1(new_n293));
  nor022aa1n16x5               g198(.a(\b[25] ), .b(\a[26] ), .o1(new_n294));
  nand42aa1n03x5               g199(.a(\b[25] ), .b(\a[26] ), .o1(new_n295));
  nanb02aa1n09x5               g200(.a(new_n294), .b(new_n295), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n289), .d(new_n290), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n290), .b(new_n288), .c(new_n204), .d(new_n280), .o1(new_n298));
  nona22aa1n03x5               g203(.a(new_n298), .b(new_n296), .c(new_n293), .out0(new_n299));
  nanp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[26] ));
  norb02aa1n06x5               g205(.a(new_n290), .b(new_n296), .out0(new_n301));
  nano32aa1d12x5               g206(.a(new_n235), .b(new_n301), .c(new_n258), .d(new_n279), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n213), .c(new_n136), .d(new_n192), .o1(new_n303));
  oai012aa1n02x5               g208(.a(new_n295), .b(new_n294), .c(new_n293), .o1(new_n304));
  aobi12aa1n03x5               g209(.a(new_n304), .b(new_n288), .c(new_n301), .out0(new_n305));
  tech160nm_fixorc02aa1n05x5   g210(.a(\a[27] ), .b(\b[26] ), .out0(new_n306));
  xnbna2aa1n03x5               g211(.a(new_n306), .b(new_n305), .c(new_n303), .out0(\s[27] ));
  aoai13aa1n03x5               g212(.a(new_n301), .b(new_n286), .c(new_n266), .d(new_n279), .o1(new_n308));
  nanp03aa1n06x5               g213(.a(new_n303), .b(new_n308), .c(new_n304), .o1(new_n309));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  norp02aa1n02x5               g215(.a(\b[27] ), .b(\a[28] ), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(\b[27] ), .b(\a[28] ), .o1(new_n312));
  nanb02aa1n02x5               g217(.a(new_n311), .b(new_n312), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n310), .c(new_n309), .d(new_n306), .o1(new_n314));
  aoai13aa1n02x5               g219(.a(new_n279), .b(new_n270), .c(new_n242), .d(new_n258), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n301), .o1(new_n316));
  aoai13aa1n02x7               g221(.a(new_n304), .b(new_n316), .c(new_n315), .d(new_n287), .o1(new_n317));
  aoai13aa1n02x5               g222(.a(new_n306), .b(new_n317), .c(new_n204), .d(new_n302), .o1(new_n318));
  nona22aa1n02x4               g223(.a(new_n318), .b(new_n313), .c(new_n310), .out0(new_n319));
  nanp02aa1n03x5               g224(.a(new_n314), .b(new_n319), .o1(\s[28] ));
  norb02aa1n06x5               g225(.a(new_n306), .b(new_n313), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n317), .c(new_n204), .d(new_n302), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .out0(new_n323));
  aoi012aa1n02x5               g228(.a(new_n311), .b(new_n310), .c(new_n312), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(new_n325));
  inv000aa1d42x5               g230(.a(new_n321), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n324), .b(new_n326), .c(new_n305), .d(new_n303), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n327), .b(new_n323), .c(new_n322), .d(new_n325), .o1(\s[29] ));
  xorb03aa1n02x5               g233(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g234(.a(new_n313), .b(new_n323), .c(new_n306), .out0(new_n330));
  nanb02aa1n03x5               g235(.a(new_n330), .b(new_n309), .out0(new_n331));
  inv000aa1d42x5               g236(.a(\b[28] ), .o1(new_n332));
  inv000aa1d42x5               g237(.a(\a[29] ), .o1(new_n333));
  oaib12aa1n02x5               g238(.a(new_n324), .b(\b[28] ), .c(new_n333), .out0(new_n334));
  oaib12aa1n02x5               g239(.a(new_n334), .b(new_n332), .c(\a[29] ), .out0(new_n335));
  aoai13aa1n02x7               g240(.a(new_n335), .b(new_n330), .c(new_n305), .d(new_n303), .o1(new_n336));
  xorc02aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .out0(new_n337));
  oaoi13aa1n02x5               g242(.a(new_n337), .b(new_n334), .c(new_n333), .d(new_n332), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n336), .b(new_n337), .c(new_n331), .d(new_n338), .o1(\s[30] ));
  nano22aa1n06x5               g244(.a(new_n326), .b(new_n323), .c(new_n337), .out0(new_n340));
  aoai13aa1n03x5               g245(.a(new_n340), .b(new_n317), .c(new_n204), .d(new_n302), .o1(new_n341));
  aoi022aa1n02x5               g246(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n342));
  norb02aa1n02x5               g247(.a(\b[30] ), .b(\a[31] ), .out0(new_n343));
  obai22aa1n02x7               g248(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n344));
  aoi112aa1n02x5               g249(.a(new_n344), .b(new_n343), .c(new_n334), .d(new_n342), .o1(new_n345));
  inv000aa1d42x5               g250(.a(new_n340), .o1(new_n346));
  norp02aa1n02x5               g251(.a(\b[29] ), .b(\a[30] ), .o1(new_n347));
  aoi012aa1n02x5               g252(.a(new_n347), .b(new_n334), .c(new_n342), .o1(new_n348));
  aoai13aa1n03x5               g253(.a(new_n348), .b(new_n346), .c(new_n305), .d(new_n303), .o1(new_n349));
  xorc02aa1n02x5               g254(.a(\a[31] ), .b(\b[30] ), .out0(new_n350));
  aoi022aa1n03x5               g255(.a(new_n349), .b(new_n350), .c(new_n345), .d(new_n341), .o1(\s[31] ));
  nanp03aa1n02x5               g256(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n113), .b(new_n352), .c(new_n104), .out0(\s[3] ));
  aoi012aa1n02x5               g258(.a(new_n111), .b(new_n107), .c(new_n112), .o1(new_n354));
  xnrc02aa1n02x5               g259(.a(new_n354), .b(new_n110), .out0(\s[4] ));
  norb02aa1n02x5               g260(.a(new_n118), .b(new_n119), .out0(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n356), .b(new_n114), .c(new_n115), .out0(\s[5] ));
  norb02aa1n02x5               g262(.a(new_n117), .b(new_n116), .out0(new_n358));
  inv000aa1d42x5               g263(.a(new_n119), .o1(new_n359));
  nanp02aa1n02x5               g264(.a(new_n114), .b(new_n115), .o1(new_n360));
  nanp02aa1n02x5               g265(.a(new_n360), .b(new_n356), .o1(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n358), .b(new_n361), .c(new_n359), .out0(\s[6] ));
  aoai13aa1n02x5               g267(.a(new_n358), .b(new_n119), .c(new_n360), .d(new_n118), .o1(new_n363));
  xnbna2aa1n03x5               g268(.a(new_n129), .b(new_n363), .c(new_n130), .out0(\s[7] ));
  aoai13aa1n02x5               g269(.a(new_n133), .b(new_n126), .c(new_n363), .d(new_n130), .o1(new_n365));
  xorb03aa1n02x5               g270(.a(new_n365), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi012aa1n02x5               g271(.a(new_n127), .b(new_n114), .c(new_n115), .o1(new_n367));
  aoi113aa1n02x5               g272(.a(new_n137), .b(new_n134), .c(new_n132), .d(new_n129), .e(new_n128), .o1(new_n368));
  aboi22aa1n03x5               g273(.a(new_n367), .b(new_n368), .c(new_n136), .d(new_n137), .out0(\s[9] ));
endmodule

