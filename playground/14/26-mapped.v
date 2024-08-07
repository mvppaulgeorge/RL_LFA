// Benchmark "adder" written by ABC on Wed Jul 17 19:21:26 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n342, new_n343, new_n346, new_n347,
    new_n348, new_n350, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[2] ), .b(\a[3] ), .out0(new_n97));
  orn002aa1n03x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nand02aa1n06x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  aob012aa1d18x5               g004(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(new_n100));
  norp02aa1n04x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand42aa1n16x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb03aa1n03x5               g008(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n104));
  aoai13aa1n06x5               g009(.a(new_n104), .b(new_n97), .c(new_n100), .d(new_n98), .o1(new_n105));
  nanp02aa1n09x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  aoi022aa1d24x5               g011(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n107), .b(new_n106), .c(\b[7] ), .d(\a[8] ), .o1(new_n108));
  nor002aa1n16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  inv000aa1n02x5               g014(.a(new_n109), .o1(new_n110));
  tech160nm_fixnrc02aa1n04x5   g015(.a(\b[4] ), .b(\a[5] ), .out0(new_n111));
  nor042aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  aoi012aa1n02x5               g017(.a(new_n112), .b(\a[4] ), .c(\b[3] ), .o1(new_n113));
  nano23aa1n06x5               g018(.a(new_n108), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  nand42aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand03aa1n04x5               g021(.a(new_n115), .b(new_n106), .c(new_n116), .o1(new_n117));
  oaih22aa1n06x5               g022(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n118));
  nand42aa1n02x5               g023(.a(new_n118), .b(new_n116), .o1(new_n119));
  nor022aa1n06x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  norb03aa1n03x5               g025(.a(new_n115), .b(new_n120), .c(new_n109), .out0(new_n121));
  oai013aa1n06x5               g026(.a(new_n119), .b(new_n121), .c(new_n117), .d(new_n118), .o1(new_n122));
  aoi012aa1n06x5               g027(.a(new_n122), .b(new_n114), .c(new_n105), .o1(new_n123));
  tech160nm_fioaoi03aa1n03p5x5 g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n06x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  tech160nm_fixorc02aa1n02p5x5 g031(.a(\a[3] ), .b(\b[2] ), .out0(new_n127));
  nanp02aa1n02x5               g032(.a(new_n100), .b(new_n98), .o1(new_n128));
  aobi12aa1n02x5               g033(.a(new_n104), .b(new_n128), .c(new_n127), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(\b[4] ), .b(\a[5] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(\a[5] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\b[4] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(new_n132), .b(new_n131), .o1(new_n133));
  nano32aa1n02x4               g038(.a(new_n112), .b(new_n133), .c(new_n130), .d(new_n102), .out0(new_n134));
  nona22aa1n02x4               g039(.a(new_n134), .b(new_n108), .c(new_n109), .out0(new_n135));
  norp03aa1n03x5               g040(.a(new_n121), .b(new_n118), .c(new_n117), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n119), .b(new_n136), .out0(new_n137));
  tech160nm_fioai012aa1n03p5x5 g042(.a(new_n137), .b(new_n135), .c(new_n129), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[8] ), .b(\a[9] ), .o1(new_n139));
  xnrc02aa1n12x5               g044(.a(\b[9] ), .b(\a[10] ), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n126), .c(new_n138), .d(new_n139), .o1(new_n142));
  norp02aa1n06x5               g047(.a(\b[9] ), .b(\a[10] ), .o1(new_n143));
  and002aa1n03x5               g048(.a(\b[9] ), .b(\a[10] ), .o(new_n144));
  oabi12aa1n02x7               g049(.a(new_n144), .b(new_n126), .c(new_n143), .out0(new_n145));
  nor002aa1d32x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand42aa1n04x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n142), .c(new_n145), .out0(\s[11] ));
  inv040aa1n03x5               g054(.a(new_n146), .o1(new_n150));
  oab012aa1n04x5               g055(.a(new_n144), .b(new_n126), .c(new_n143), .out0(new_n151));
  aoai13aa1n06x5               g056(.a(new_n148), .b(new_n151), .c(new_n124), .d(new_n141), .o1(new_n152));
  nor022aa1n08x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanp02aa1n04x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoi012aa1n03x5               g061(.a(new_n156), .b(new_n152), .c(new_n150), .o1(new_n157));
  nona22aa1n03x5               g062(.a(new_n152), .b(new_n155), .c(new_n146), .out0(new_n158));
  norb02aa1n03x4               g063(.a(new_n158), .b(new_n157), .out0(\s[12] ));
  inv000aa1n02x5               g064(.a(new_n126), .o1(new_n160));
  nona23aa1n09x5               g065(.a(new_n154), .b(new_n147), .c(new_n146), .d(new_n153), .out0(new_n161));
  nano23aa1n02x4               g066(.a(new_n161), .b(new_n140), .c(new_n139), .d(new_n160), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n122), .c(new_n114), .d(new_n105), .o1(new_n163));
  nano23aa1n09x5               g068(.a(new_n146), .b(new_n153), .c(new_n154), .d(new_n147), .out0(new_n164));
  oaoi03aa1n02x5               g069(.a(\a[12] ), .b(\b[11] ), .c(new_n150), .o1(new_n165));
  tech160nm_fiaoi012aa1n04x5   g070(.a(new_n165), .b(new_n164), .c(new_n151), .o1(new_n166));
  nor002aa1d32x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  tech160nm_finand02aa1n05x5   g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n163), .c(new_n166), .out0(\s[13] ));
  inv040aa1n08x5               g075(.a(new_n167), .o1(new_n171));
  inv000aa1n02x5               g076(.a(new_n169), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n171), .b(new_n172), .c(new_n163), .d(new_n166), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oaoi03aa1n12x5               g079(.a(\a[14] ), .b(\b[13] ), .c(new_n171), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  nor022aa1n16x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  nona23aa1n02x5               g083(.a(new_n178), .b(new_n168), .c(new_n167), .d(new_n177), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n176), .b(new_n179), .c(new_n163), .d(new_n166), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n04x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand42aa1n10x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  tech160nm_finor002aa1n05x5   g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nand42aa1n06x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  aoai13aa1n03x5               g091(.a(new_n186), .b(new_n182), .c(new_n180), .d(new_n183), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n182), .b(new_n186), .c(new_n180), .d(new_n183), .o1(new_n188));
  norb02aa1n02x7               g093(.a(new_n187), .b(new_n188), .out0(\s[16] ));
  nano22aa1n09x5               g094(.a(new_n140), .b(new_n160), .c(new_n139), .out0(new_n190));
  nano23aa1n06x5               g095(.a(new_n167), .b(new_n177), .c(new_n178), .d(new_n168), .out0(new_n191));
  nano23aa1n09x5               g096(.a(new_n182), .b(new_n184), .c(new_n185), .d(new_n183), .out0(new_n192));
  nanp02aa1n04x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nano22aa1n03x7               g098(.a(new_n193), .b(new_n190), .c(new_n164), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n122), .c(new_n105), .d(new_n114), .o1(new_n195));
  oabi12aa1n06x5               g100(.a(new_n165), .b(new_n161), .c(new_n145), .out0(new_n196));
  aoi112aa1n02x5               g101(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n197));
  nanp02aa1n03x5               g102(.a(new_n192), .b(new_n175), .o1(new_n198));
  nona22aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n184), .out0(new_n199));
  aoib12aa1n12x5               g104(.a(new_n199), .b(new_n196), .c(new_n193), .out0(new_n200));
  xnrc02aa1n12x5               g105(.a(\b[16] ), .b(\a[17] ), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n200), .out0(\s[17] ));
  inv040aa1d32x5               g107(.a(\a[17] ), .o1(new_n203));
  inv030aa1n24x5               g108(.a(\b[16] ), .o1(new_n204));
  nona23aa1n09x5               g109(.a(new_n192), .b(new_n190), .c(new_n161), .d(new_n179), .out0(new_n205));
  oai012aa1n12x5               g110(.a(new_n200), .b(new_n123), .c(new_n205), .o1(new_n206));
  oaoi03aa1n03x5               g111(.a(new_n203), .b(new_n204), .c(new_n206), .o1(new_n207));
  xnrb03aa1n03x5               g112(.a(new_n207), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1d32x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nand42aa1d28x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  norb03aa1n03x5               g115(.a(new_n210), .b(new_n201), .c(new_n209), .out0(new_n211));
  inv030aa1n02x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n12x5               g117(.a(new_n210), .b(new_n209), .c(new_n203), .d(new_n204), .o1(new_n213));
  aoai13aa1n04x5               g118(.a(new_n213), .b(new_n212), .c(new_n195), .d(new_n200), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n06x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  inv040aa1n03x5               g122(.a(new_n217), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n213), .o1(new_n219));
  nand42aa1n06x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nanb02aa1n12x5               g125(.a(new_n217), .b(new_n220), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n219), .c(new_n206), .d(new_n211), .o1(new_n223));
  inv040aa1d32x5               g128(.a(\a[20] ), .o1(new_n224));
  inv040aa1d32x5               g129(.a(\b[19] ), .o1(new_n225));
  nand42aa1d28x5               g130(.a(new_n225), .b(new_n224), .o1(new_n226));
  nand02aa1d28x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand02aa1d28x5               g132(.a(new_n226), .b(new_n227), .o1(new_n228));
  aoi012aa1n03x5               g133(.a(new_n228), .b(new_n223), .c(new_n218), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n228), .o1(new_n230));
  aoi112aa1n03x4               g135(.a(new_n217), .b(new_n230), .c(new_n214), .d(new_n222), .o1(new_n231));
  norp02aa1n03x5               g136(.a(new_n229), .b(new_n231), .o1(\s[20] ));
  nano22aa1d15x5               g137(.a(new_n228), .b(new_n218), .c(new_n220), .out0(new_n233));
  nona23aa1d16x5               g138(.a(new_n233), .b(new_n210), .c(new_n201), .d(new_n209), .out0(new_n234));
  aoi112aa1n02x7               g139(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  nor043aa1d12x5               g141(.a(new_n213), .b(new_n221), .c(new_n228), .o1(new_n237));
  nano22aa1d15x5               g142(.a(new_n237), .b(new_n226), .c(new_n236), .out0(new_n238));
  aoai13aa1n04x5               g143(.a(new_n238), .b(new_n234), .c(new_n195), .d(new_n200), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n234), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n238), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[21] ), .b(\b[20] ), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n244), .c(new_n206), .d(new_n243), .o1(new_n246));
  xorc02aa1n02x5               g151(.a(\a[22] ), .b(\b[21] ), .out0(new_n247));
  aobi12aa1n03x5               g152(.a(new_n247), .b(new_n246), .c(new_n242), .out0(new_n248));
  aoi112aa1n03x4               g153(.a(new_n241), .b(new_n247), .c(new_n239), .d(new_n245), .o1(new_n249));
  nor002aa1n02x5               g154(.a(new_n248), .b(new_n249), .o1(\s[22] ));
  inv030aa1d32x5               g155(.a(\a[21] ), .o1(new_n251));
  inv040aa1d32x5               g156(.a(\a[22] ), .o1(new_n252));
  xroi22aa1d06x4               g157(.a(new_n251), .b(\b[20] ), .c(new_n252), .d(\b[21] ), .out0(new_n253));
  nano22aa1n03x5               g158(.a(new_n212), .b(new_n233), .c(new_n253), .out0(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  inv000aa1n02x5               g160(.a(new_n253), .o1(new_n256));
  oaih22aa1d12x5               g161(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n257));
  oaib12aa1n09x5               g162(.a(new_n257), .b(new_n252), .c(\b[21] ), .out0(new_n258));
  oa0012aa1n03x5               g163(.a(new_n258), .b(new_n238), .c(new_n256), .o(new_n259));
  aoai13aa1n04x5               g164(.a(new_n259), .b(new_n255), .c(new_n195), .d(new_n200), .o1(new_n260));
  xorb03aa1n02x5               g165(.a(new_n260), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  inv040aa1n03x5               g168(.a(new_n259), .o1(new_n264));
  xorc02aa1n02x5               g169(.a(\a[23] ), .b(\b[22] ), .out0(new_n265));
  aoai13aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n206), .d(new_n254), .o1(new_n266));
  nor002aa1d32x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  nand42aa1n08x5               g172(.a(\b[23] ), .b(\a[24] ), .o1(new_n268));
  nanb02aa1n12x5               g173(.a(new_n267), .b(new_n268), .out0(new_n269));
  aoi012aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n263), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n268), .b(new_n267), .out0(new_n271));
  aoi112aa1n03x4               g176(.a(new_n262), .b(new_n271), .c(new_n260), .d(new_n265), .o1(new_n272));
  nor002aa1n02x5               g177(.a(new_n270), .b(new_n272), .o1(\s[24] ));
  xnrc02aa1n02x5               g178(.a(\b[22] ), .b(\a[23] ), .out0(new_n274));
  nor002aa1n02x5               g179(.a(new_n274), .b(new_n269), .o1(new_n275));
  nano22aa1n03x7               g180(.a(new_n234), .b(new_n253), .c(new_n275), .out0(new_n276));
  inv000aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  nand02aa1n02x5               g182(.a(new_n253), .b(new_n275), .o1(new_n278));
  norp03aa1n02x5               g183(.a(new_n274), .b(new_n258), .c(new_n269), .o1(new_n279));
  aoi112aa1n06x5               g184(.a(new_n279), .b(new_n267), .c(new_n268), .d(new_n262), .o1(new_n280));
  oai012aa1d24x5               g185(.a(new_n280), .b(new_n238), .c(new_n278), .o1(new_n281));
  inv030aa1n04x5               g186(.a(new_n281), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n277), .c(new_n195), .d(new_n200), .o1(new_n283));
  xorb03aa1n02x5               g188(.a(new_n283), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  inv000aa1n02x5               g190(.a(new_n285), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[25] ), .b(\b[24] ), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n281), .c(new_n206), .d(new_n276), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  aobi12aa1n03x5               g194(.a(new_n289), .b(new_n288), .c(new_n286), .out0(new_n290));
  aoi112aa1n03x5               g195(.a(new_n285), .b(new_n289), .c(new_n283), .d(new_n287), .o1(new_n291));
  nor002aa1n02x5               g196(.a(new_n290), .b(new_n291), .o1(\s[26] ));
  aoi112aa1n02x7               g197(.a(new_n197), .b(new_n184), .c(new_n192), .d(new_n175), .o1(new_n293));
  oai012aa1n02x5               g198(.a(new_n293), .b(new_n166), .c(new_n193), .o1(new_n294));
  and002aa1n24x5               g199(.a(new_n289), .b(new_n287), .o(new_n295));
  nano32aa1n03x7               g200(.a(new_n234), .b(new_n295), .c(new_n253), .d(new_n275), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n294), .c(new_n138), .d(new_n194), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n298));
  aobi12aa1n18x5               g203(.a(new_n298), .b(new_n281), .c(new_n295), .out0(new_n299));
  xorc02aa1n12x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  xnbna2aa1n03x5               g205(.a(new_n300), .b(new_n297), .c(new_n299), .out0(\s[27] ));
  norp02aa1n02x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  inv040aa1n03x5               g207(.a(new_n302), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n226), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n265), .b(new_n271), .o1(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n245), .c(new_n247), .out0(new_n306));
  oai013aa1n03x5               g211(.a(new_n306), .b(new_n237), .c(new_n304), .d(new_n235), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n295), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n298), .b(new_n308), .c(new_n307), .d(new_n280), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n300), .b(new_n309), .c(new_n206), .d(new_n296), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[27] ), .b(\a[28] ), .out0(new_n311));
  aoi012aa1n03x5               g216(.a(new_n311), .b(new_n310), .c(new_n303), .o1(new_n312));
  aobi12aa1n06x5               g217(.a(new_n300), .b(new_n297), .c(new_n299), .out0(new_n313));
  nano22aa1n03x7               g218(.a(new_n313), .b(new_n303), .c(new_n311), .out0(new_n314));
  norp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[28] ));
  norb02aa1n02x5               g220(.a(new_n300), .b(new_n311), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n309), .c(new_n206), .d(new_n296), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n303), .carry(new_n318));
  xnrc02aa1n02x5               g223(.a(\b[28] ), .b(\a[29] ), .out0(new_n319));
  aoi012aa1n03x5               g224(.a(new_n319), .b(new_n317), .c(new_n318), .o1(new_n320));
  aobi12aa1n06x5               g225(.a(new_n316), .b(new_n297), .c(new_n299), .out0(new_n321));
  nano22aa1n03x7               g226(.a(new_n321), .b(new_n318), .c(new_n319), .out0(new_n322));
  norp02aa1n03x5               g227(.a(new_n320), .b(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g228(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g229(.a(new_n300), .b(new_n319), .c(new_n311), .out0(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n309), .c(new_n206), .d(new_n296), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .c(new_n318), .carry(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .out0(new_n328));
  aoi012aa1n03x5               g233(.a(new_n328), .b(new_n326), .c(new_n327), .o1(new_n329));
  aobi12aa1n06x5               g234(.a(new_n325), .b(new_n297), .c(new_n299), .out0(new_n330));
  nano22aa1n03x7               g235(.a(new_n330), .b(new_n327), .c(new_n328), .out0(new_n331));
  nor002aa1n02x5               g236(.a(new_n329), .b(new_n331), .o1(\s[30] ));
  norb02aa1n02x5               g237(.a(new_n325), .b(new_n328), .out0(new_n333));
  aobi12aa1n06x5               g238(.a(new_n333), .b(new_n297), .c(new_n299), .out0(new_n334));
  oao003aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .c(new_n327), .carry(new_n335));
  xnrc02aa1n02x5               g240(.a(\b[30] ), .b(\a[31] ), .out0(new_n336));
  nano22aa1n03x7               g241(.a(new_n334), .b(new_n335), .c(new_n336), .out0(new_n337));
  aoai13aa1n03x5               g242(.a(new_n333), .b(new_n309), .c(new_n206), .d(new_n296), .o1(new_n338));
  aoi012aa1n03x5               g243(.a(new_n336), .b(new_n338), .c(new_n335), .o1(new_n339));
  nor002aa1n02x5               g244(.a(new_n339), .b(new_n337), .o1(\s[31] ));
  xnbna2aa1n03x5               g245(.a(new_n127), .b(new_n100), .c(new_n98), .out0(\s[3] ));
  norb02aa1n02x5               g246(.a(new_n102), .b(new_n103), .out0(new_n342));
  aoi012aa1n02x5               g247(.a(new_n101), .b(new_n128), .c(new_n127), .o1(new_n343));
  oai012aa1n02x5               g248(.a(new_n105), .b(new_n343), .c(new_n342), .o1(\s[4] ));
  xnbna2aa1n03x5               g249(.a(new_n111), .b(new_n105), .c(new_n102), .out0(\s[5] ));
  aoi013aa1n02x4               g250(.a(new_n120), .b(new_n105), .c(new_n130), .d(new_n102), .o1(new_n346));
  nanb03aa1n03x5               g251(.a(new_n111), .b(new_n105), .c(new_n102), .out0(new_n347));
  nanp02aa1n03x5               g252(.a(new_n347), .b(new_n121), .o1(new_n348));
  aoai13aa1n02x5               g253(.a(new_n348), .b(new_n346), .c(new_n115), .d(new_n110), .o1(\s[6] ));
  aoi022aa1n02x5               g254(.a(new_n347), .b(new_n121), .c(\a[6] ), .d(\b[5] ), .o1(new_n350));
  xorb03aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi013aa1n03x5               g256(.a(new_n112), .b(new_n348), .c(new_n106), .d(new_n115), .o1(new_n352));
  xnrb03aa1n03x5               g257(.a(new_n352), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g258(.a(new_n123), .b(new_n139), .c(new_n160), .out0(\s[9] ));
endmodule


