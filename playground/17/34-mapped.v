// Benchmark "adder" written by ABC on Wed Jul 17 20:58:39 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n341, new_n342, new_n344, new_n346,
    new_n347, new_n348, new_n350, new_n351, new_n352, new_n354, new_n355,
    new_n356, new_n359, new_n360;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n24x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n09x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  norb03aa1n12x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  nanp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  orn002aa1n24x5               g008(.a(\a[3] ), .b(\b[2] ), .o(new_n104));
  nanp03aa1d12x5               g009(.a(new_n104), .b(new_n100), .c(new_n103), .o1(new_n105));
  orn002aa1n24x5               g010(.a(\a[4] ), .b(\b[3] ), .o(new_n106));
  nand42aa1n10x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n106), .b(new_n107), .c(\b[2] ), .d(\a[3] ), .o1(new_n108));
  oabi12aa1n18x5               g013(.a(new_n108), .b(new_n105), .c(new_n102), .out0(new_n109));
  inv040aa1d32x5               g014(.a(\a[5] ), .o1(new_n110));
  inv040aa1d32x5               g015(.a(\b[4] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  inv040aa1n06x5               g017(.a(new_n112), .o1(new_n113));
  oai122aa1n12x5               g018(.a(new_n113), .b(\a[6] ), .c(\b[5] ), .d(new_n110), .e(new_n111), .o1(new_n114));
  aoi022aa1d18x5               g019(.a(new_n111), .b(new_n110), .c(\a[4] ), .d(\b[3] ), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nanp02aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanb02aa1n06x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  aoi022aa1d24x5               g023(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n119));
  nano23aa1d15x5               g024(.a(new_n114), .b(new_n118), .c(new_n115), .d(new_n119), .out0(new_n120));
  tech160nm_fioai012aa1n05x5   g025(.a(new_n117), .b(new_n116), .c(new_n112), .o1(new_n121));
  norp02aa1n04x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  and002aa1n12x5               g027(.a(\b[5] ), .b(\a[6] ), .o(new_n123));
  aoi112aa1n09x5               g028(.a(new_n123), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n124));
  nona23aa1d18x5               g029(.a(new_n119), .b(new_n117), .c(new_n112), .d(new_n116), .out0(new_n125));
  oai012aa1n12x5               g030(.a(new_n121), .b(new_n125), .c(new_n124), .o1(new_n126));
  nand42aa1n10x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n120), .d(new_n109), .o1(new_n129));
  nor002aa1n08x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oai022aa1d24x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  nanb03aa1n02x5               g038(.a(new_n133), .b(new_n129), .c(new_n131), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n98), .d(new_n129), .o1(\s[10] ));
  nano23aa1n09x5               g040(.a(new_n97), .b(new_n130), .c(new_n131), .d(new_n127), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n126), .c(new_n120), .d(new_n109), .o1(new_n137));
  oai012aa1n02x5               g042(.a(new_n131), .b(new_n130), .c(new_n97), .o1(new_n138));
  nor022aa1n16x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand02aa1d28x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n06x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n137), .c(new_n138), .out0(\s[11] ));
  inv000aa1d42x5               g047(.a(\b[10] ), .o1(new_n143));
  nanb02aa1d36x5               g048(.a(\a[11] ), .b(new_n143), .out0(new_n144));
  aob012aa1n02x5               g049(.a(new_n141), .b(new_n137), .c(new_n138), .out0(new_n145));
  nor002aa1d32x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand02aa1d28x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n06x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  nona23aa1n02x4               g053(.a(new_n145), .b(new_n147), .c(new_n146), .d(new_n139), .out0(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n144), .d(new_n145), .o1(\s[12] ));
  nand02aa1d08x5               g055(.a(new_n120), .b(new_n109), .o1(new_n151));
  inv030aa1n06x5               g056(.a(new_n126), .o1(new_n152));
  nand02aa1d06x5               g057(.a(new_n151), .b(new_n152), .o1(new_n153));
  nona23aa1n09x5               g058(.a(new_n131), .b(new_n127), .c(new_n97), .d(new_n130), .out0(new_n154));
  nano22aa1n03x7               g059(.a(new_n154), .b(new_n141), .c(new_n148), .out0(new_n155));
  oaih12aa1n06x5               g060(.a(new_n147), .b(new_n146), .c(new_n139), .o1(new_n156));
  nanb03aa1d24x5               g061(.a(new_n146), .b(new_n147), .c(new_n140), .out0(new_n157));
  nand23aa1d12x5               g062(.a(new_n133), .b(new_n144), .c(new_n131), .o1(new_n158));
  oai012aa1n18x5               g063(.a(new_n156), .b(new_n158), .c(new_n157), .o1(new_n159));
  nor002aa1d32x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand42aa1d28x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n03x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n159), .c(new_n153), .d(new_n155), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n162), .o1(new_n164));
  oai112aa1n02x5               g069(.a(new_n156), .b(new_n164), .c(new_n158), .d(new_n157), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n165), .b(new_n153), .c(new_n155), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n163), .b(new_n166), .out0(\s[13] ));
  inv030aa1n02x5               g072(.a(new_n160), .o1(new_n168));
  nor002aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1n20x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  nona23aa1n03x5               g076(.a(new_n163), .b(new_n170), .c(new_n169), .d(new_n160), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n168), .d(new_n163), .o1(\s[14] ));
  nand03aa1n02x5               g078(.a(new_n136), .b(new_n141), .c(new_n148), .o1(new_n174));
  nona23aa1n09x5               g079(.a(new_n170), .b(new_n161), .c(new_n160), .d(new_n169), .out0(new_n175));
  nona22aa1n03x5               g080(.a(new_n153), .b(new_n174), .c(new_n175), .out0(new_n176));
  nano23aa1d15x5               g081(.a(new_n160), .b(new_n169), .c(new_n170), .d(new_n161), .out0(new_n177));
  oaoi03aa1n09x5               g082(.a(\a[14] ), .b(\b[13] ), .c(new_n168), .o1(new_n178));
  aoi012aa1n02x5               g083(.a(new_n178), .b(new_n159), .c(new_n177), .o1(new_n179));
  xorc02aa1n12x5               g084(.a(\a[15] ), .b(\b[14] ), .out0(new_n180));
  aob012aa1n03x5               g085(.a(new_n180), .b(new_n176), .c(new_n179), .out0(new_n181));
  aoi112aa1n02x5               g086(.a(new_n180), .b(new_n178), .c(new_n159), .d(new_n177), .o1(new_n182));
  aobi12aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n176), .out0(\s[15] ));
  norp02aa1n02x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  inv000aa1n03x5               g089(.a(new_n184), .o1(new_n185));
  tech160nm_fixorc02aa1n05x5   g090(.a(\a[16] ), .b(\b[15] ), .out0(new_n186));
  and002aa1n02x5               g091(.a(\b[15] ), .b(\a[16] ), .o(new_n187));
  oai022aa1n02x5               g092(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n188));
  nona22aa1n03x5               g093(.a(new_n181), .b(new_n187), .c(new_n188), .out0(new_n189));
  aoai13aa1n03x5               g094(.a(new_n189), .b(new_n186), .c(new_n181), .d(new_n185), .o1(\s[16] ));
  nano32aa1n03x7               g095(.a(new_n174), .b(new_n186), .c(new_n177), .d(new_n180), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n126), .c(new_n109), .d(new_n120), .o1(new_n192));
  and002aa1n06x5               g097(.a(new_n186), .b(new_n180), .o(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n178), .c(new_n159), .d(new_n177), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(\a[16] ), .b(\b[15] ), .c(new_n185), .o1(new_n195));
  inv000aa1n02x5               g100(.a(new_n195), .o1(new_n196));
  nand23aa1n04x5               g101(.a(new_n192), .b(new_n194), .c(new_n196), .o1(new_n197));
  xorc02aa1n12x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  nand23aa1n03x5               g103(.a(new_n193), .b(new_n155), .c(new_n177), .o1(new_n199));
  aoi012aa1d24x5               g104(.a(new_n199), .b(new_n151), .c(new_n152), .o1(new_n200));
  nano23aa1n02x4               g105(.a(new_n200), .b(new_n198), .c(new_n194), .d(new_n196), .out0(new_n201));
  aoi012aa1n02x5               g106(.a(new_n201), .b(new_n197), .c(new_n198), .o1(\s[17] ));
  nand02aa1d08x5               g107(.a(new_n194), .b(new_n196), .o1(new_n203));
  norp02aa1n02x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  oaoi13aa1n02x5               g109(.a(new_n204), .b(new_n198), .c(new_n203), .d(new_n200), .o1(new_n205));
  tech160nm_fixorc02aa1n05x5   g110(.a(\a[18] ), .b(\b[17] ), .out0(new_n206));
  oai022aa1n12x5               g111(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n207));
  aoi122aa1n06x5               g112(.a(new_n207), .b(\b[17] ), .c(\a[18] ), .d(new_n197), .e(new_n198), .o1(new_n208));
  oabi12aa1n03x5               g113(.a(new_n208), .b(new_n205), .c(new_n206), .out0(\s[18] ));
  and002aa1n02x5               g114(.a(new_n206), .b(new_n198), .o(new_n210));
  oai012aa1n06x5               g115(.a(new_n210), .b(new_n203), .c(new_n200), .o1(new_n211));
  inv000aa1d42x5               g116(.a(\a[18] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[17] ), .o1(new_n213));
  oao003aa1n02x5               g118(.a(new_n212), .b(new_n213), .c(new_n204), .carry(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  tech160nm_fixorc02aa1n03p5x5 g120(.a(\a[19] ), .b(\b[18] ), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n211), .c(new_n215), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g123(.a(\a[19] ), .o1(new_n219));
  nanb02aa1n02x5               g124(.a(\b[18] ), .b(new_n219), .out0(new_n220));
  aoai13aa1n03x5               g125(.a(new_n216), .b(new_n214), .c(new_n197), .d(new_n210), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[20] ), .b(\b[19] ), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n216), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  oai022aa1d18x5               g129(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  aoai13aa1n02x7               g131(.a(new_n226), .b(new_n223), .c(new_n211), .d(new_n215), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n221), .d(new_n220), .o1(\s[20] ));
  inv020aa1n04x5               g133(.a(\a[20] ), .o1(new_n229));
  xroi22aa1d06x4               g134(.a(new_n219), .b(\b[18] ), .c(new_n229), .d(\b[19] ), .out0(new_n230));
  nand23aa1d12x5               g135(.a(new_n230), .b(new_n198), .c(new_n206), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  oai012aa1n06x5               g137(.a(new_n232), .b(new_n203), .c(new_n200), .o1(new_n233));
  aoi022aa1d18x5               g138(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n234));
  aoi012aa1d18x5               g139(.a(new_n225), .b(new_n207), .c(new_n234), .o1(new_n235));
  norb02aa1n03x5               g140(.a(new_n224), .b(new_n235), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[21] ), .b(\b[20] ), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n233), .c(new_n237), .out0(\s[21] ));
  nor002aa1d32x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n238), .b(new_n236), .c(new_n197), .d(new_n232), .o1(new_n242));
  nor002aa1d32x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  tech160nm_finand02aa1n05x5   g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n238), .o1(new_n246));
  norb03aa1n02x5               g151(.a(new_n244), .b(new_n240), .c(new_n243), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n246), .c(new_n233), .d(new_n237), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n245), .c(new_n242), .d(new_n241), .o1(\s[22] ));
  and002aa1n06x5               g154(.a(new_n238), .b(new_n245), .o(new_n250));
  norb02aa1n02x7               g155(.a(new_n250), .b(new_n231), .out0(new_n251));
  oai012aa1n06x5               g156(.a(new_n251), .b(new_n203), .c(new_n200), .o1(new_n252));
  tech160nm_fiaoi012aa1n04x5   g157(.a(new_n240), .b(\a[20] ), .c(\b[19] ), .o1(new_n253));
  tech160nm_fiaoi012aa1n05x5   g158(.a(new_n243), .b(\a[21] ), .c(\b[20] ), .o1(new_n254));
  nand23aa1n06x5               g159(.a(new_n253), .b(new_n254), .c(new_n244), .o1(new_n255));
  tech160nm_fioai012aa1n05x5   g160(.a(new_n244), .b(new_n243), .c(new_n240), .o1(new_n256));
  oai012aa1n18x5               g161(.a(new_n256), .b(new_n255), .c(new_n235), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n252), .b(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  oai112aa1n02x5               g166(.a(new_n256), .b(new_n261), .c(new_n255), .d(new_n235), .o1(new_n262));
  aboi22aa1n03x5               g167(.a(new_n262), .b(new_n252), .c(new_n259), .d(new_n260), .out0(\s[23] ));
  nor042aa1n06x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n260), .b(new_n257), .c(new_n197), .d(new_n251), .o1(new_n266));
  tech160nm_fixorc02aa1n04x5   g171(.a(\a[24] ), .b(\b[23] ), .out0(new_n267));
  oai022aa1n02x5               g172(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n268));
  aoi012aa1n02x5               g173(.a(new_n268), .b(\a[24] ), .c(\b[23] ), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n261), .c(new_n252), .d(new_n258), .o1(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n267), .c(new_n266), .d(new_n265), .o1(\s[24] ));
  nand02aa1d08x5               g176(.a(new_n267), .b(new_n260), .o1(new_n272));
  inv000aa1n06x5               g177(.a(new_n272), .o1(new_n273));
  nano22aa1n06x5               g178(.a(new_n231), .b(new_n273), .c(new_n250), .out0(new_n274));
  tech160nm_fioai012aa1n05x5   g179(.a(new_n274), .b(new_n203), .c(new_n200), .o1(new_n275));
  tech160nm_fioaoi03aa1n03p5x5 g180(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .o1(new_n276));
  tech160nm_fiaoi012aa1n05x5   g181(.a(new_n276), .b(new_n257), .c(new_n273), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n278), .c(new_n197), .d(new_n274), .o1(new_n280));
  aoi112aa1n02x5               g185(.a(new_n279), .b(new_n276), .c(new_n257), .d(new_n273), .o1(new_n281));
  aobi12aa1n02x7               g186(.a(new_n280), .b(new_n281), .c(new_n275), .out0(\s[25] ));
  nor042aa1n03x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n279), .o1(new_n286));
  oai022aa1n02x5               g191(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n287), .b(\a[26] ), .c(\b[25] ), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n275), .d(new_n277), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n285), .c(new_n280), .d(new_n284), .o1(\s[26] ));
  and002aa1n12x5               g195(.a(new_n285), .b(new_n279), .o(new_n291));
  nano32aa1n03x7               g196(.a(new_n231), .b(new_n291), .c(new_n250), .d(new_n273), .out0(new_n292));
  oai012aa1n06x5               g197(.a(new_n292), .b(new_n203), .c(new_n200), .o1(new_n293));
  oaoi13aa1n02x5               g198(.a(new_n175), .b(new_n156), .c(new_n158), .d(new_n157), .o1(new_n294));
  oaoi13aa1n02x7               g199(.a(new_n195), .b(new_n193), .c(new_n294), .d(new_n178), .o1(new_n295));
  inv020aa1n03x5               g200(.a(new_n292), .o1(new_n296));
  oaoi13aa1n06x5               g201(.a(new_n272), .b(new_n256), .c(new_n255), .d(new_n235), .o1(new_n297));
  oaoi03aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .o1(new_n298));
  oaoi13aa1n09x5               g203(.a(new_n298), .b(new_n291), .c(new_n297), .d(new_n276), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n296), .c(new_n295), .d(new_n192), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  aoi112aa1n03x4               g206(.a(new_n301), .b(new_n298), .c(new_n278), .d(new_n291), .o1(new_n302));
  aoi022aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .d(new_n293), .o1(\s[27] ));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n304), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(new_n300), .b(new_n301), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n301), .o1(new_n308));
  oai022aa1d24x5               g213(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n309), .b(\a[28] ), .c(\b[27] ), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n308), .c(new_n293), .d(new_n299), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n306), .d(new_n305), .o1(\s[28] ));
  and002aa1n02x5               g217(.a(new_n307), .b(new_n301), .o(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  inv000aa1d42x5               g219(.a(\b[27] ), .o1(new_n315));
  oaib12aa1n18x5               g220(.a(new_n309), .b(new_n315), .c(\a[28] ), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n316), .o1(new_n317));
  tech160nm_fixorc02aa1n03p5x5 g222(.a(\a[29] ), .b(\b[28] ), .out0(new_n318));
  norb02aa1n02x5               g223(.a(new_n318), .b(new_n317), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n319), .b(new_n314), .c(new_n293), .d(new_n299), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n318), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n317), .c(new_n300), .d(new_n313), .o1(new_n322));
  nanp02aa1n03x5               g227(.a(new_n322), .b(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g228(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g229(.a(new_n321), .b(new_n301), .c(new_n307), .out0(new_n325));
  tech160nm_fioaoi03aa1n03p5x5 g230(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .o1(new_n326));
  xnrc02aa1n02x5               g231(.a(\b[29] ), .b(\a[30] ), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n326), .c(new_n300), .d(new_n325), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n325), .o1(new_n329));
  norp02aa1n03x5               g234(.a(new_n326), .b(new_n327), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n293), .d(new_n299), .o1(new_n331));
  nanp02aa1n03x5               g236(.a(new_n328), .b(new_n331), .o1(\s[30] ));
  nano32aa1n03x7               g237(.a(new_n327), .b(new_n318), .c(new_n307), .d(new_n301), .out0(new_n333));
  aoi012aa1n02x5               g238(.a(new_n330), .b(\a[30] ), .c(\b[29] ), .o1(new_n334));
  xnrc02aa1n02x5               g239(.a(\b[30] ), .b(\a[31] ), .out0(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n334), .c(new_n300), .d(new_n333), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n333), .o1(new_n337));
  norp02aa1n02x5               g242(.a(new_n334), .b(new_n335), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n293), .d(new_n299), .o1(new_n339));
  nanp02aa1n03x5               g244(.a(new_n336), .b(new_n339), .o1(\s[31] ));
  norp02aa1n02x5               g245(.a(new_n105), .b(new_n102), .o1(new_n341));
  aboi22aa1n03x5               g246(.a(new_n102), .b(new_n100), .c(new_n104), .d(new_n103), .out0(new_n342));
  norp02aa1n02x5               g247(.a(new_n342), .b(new_n341), .o1(\s[3] ));
  norb02aa1n02x5               g248(.a(new_n104), .b(new_n341), .out0(new_n344));
  aoai13aa1n02x5               g249(.a(new_n109), .b(new_n344), .c(new_n107), .d(new_n106), .o1(\s[4] ));
  nanp02aa1n02x5               g250(.a(\b[4] ), .b(\a[5] ), .o1(new_n346));
  nanp02aa1n02x5               g251(.a(new_n111), .b(new_n110), .o1(new_n347));
  aoi022aa1n02x5               g252(.a(new_n109), .b(new_n107), .c(new_n347), .d(new_n346), .o1(new_n348));
  aoi013aa1n02x4               g253(.a(new_n348), .b(new_n115), .c(new_n346), .d(new_n109), .o1(\s[5] ));
  oai112aa1n06x5               g254(.a(new_n109), .b(new_n115), .c(new_n111), .d(new_n110), .o1(new_n350));
  norp02aa1n02x5               g255(.a(new_n123), .b(new_n122), .o1(new_n351));
  nanp02aa1n03x5               g256(.a(new_n350), .b(new_n124), .o1(new_n352));
  aoai13aa1n02x5               g257(.a(new_n352), .b(new_n351), .c(new_n347), .d(new_n350), .o1(\s[6] ));
  nanp03aa1n03x5               g258(.a(new_n352), .b(new_n113), .c(new_n119), .o1(new_n354));
  xnrc02aa1n02x5               g259(.a(\b[6] ), .b(\a[7] ), .out0(new_n355));
  aoai13aa1n02x5               g260(.a(new_n355), .b(new_n123), .c(new_n350), .d(new_n124), .o1(new_n356));
  and002aa1n02x5               g261(.a(new_n354), .b(new_n356), .o(\s[7] ));
  xobna2aa1n03x5               g262(.a(new_n118), .b(new_n354), .c(new_n113), .out0(\s[8] ));
  oaib12aa1n02x5               g263(.a(new_n121), .b(new_n97), .c(new_n127), .out0(new_n359));
  oab012aa1n02x4               g264(.a(new_n359), .b(new_n125), .c(new_n124), .out0(new_n360));
  aoi022aa1n02x5               g265(.a(new_n153), .b(new_n128), .c(new_n151), .d(new_n360), .o1(\s[9] ));
endmodule


