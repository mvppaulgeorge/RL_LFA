// Benchmark "adder" written by ABC on Wed Jul 17 19:14:39 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n301, new_n302,
    new_n303, new_n304, new_n305, new_n306, new_n307, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n320, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n329, new_n330, new_n331, new_n332,
    new_n333, new_n335, new_n336, new_n337, new_n338, new_n339, new_n340,
    new_n341, new_n343, new_n345, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n353, new_n354, new_n355, new_n356, new_n357,
    new_n358, new_n359, new_n362, new_n363, new_n365, new_n367, new_n368,
    new_n369, new_n370, new_n372, new_n373, new_n375, new_n376, new_n378;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor022aa1n04x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nand42aa1n04x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  norb03aa1n02x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  nand42aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n03x5               g008(.a(new_n99), .b(new_n103), .out0(new_n104));
  oai112aa1n06x5               g009(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  oai012aa1n06x5               g012(.a(new_n102), .b(new_n107), .c(new_n104), .o1(new_n108));
  nor042aa1n06x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nor042aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n08x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nona22aa1n09x5               g016(.a(new_n111), .b(new_n110), .c(new_n109), .out0(new_n112));
  aob012aa1n06x5               g017(.a(new_n100), .b(\b[4] ), .c(\a[5] ), .out0(new_n113));
  aoi022aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  oai122aa1n02x7               g019(.a(new_n114), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n115));
  nor043aa1n03x5               g020(.a(new_n115), .b(new_n112), .c(new_n113), .o1(new_n116));
  inv000aa1n06x5               g021(.a(new_n109), .o1(new_n117));
  oaoi03aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .c(new_n117), .o1(new_n118));
  oai022aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  norb02aa1n02x5               g025(.a(new_n120), .b(new_n119), .out0(new_n121));
  nona23aa1n09x5               g026(.a(new_n114), .b(new_n111), .c(new_n109), .d(new_n110), .out0(new_n122));
  oabi12aa1n06x5               g027(.a(new_n118), .b(new_n122), .c(new_n121), .out0(new_n123));
  tech160nm_fixorc02aa1n03p5x5 g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n116), .d(new_n108), .o1(new_n125));
  nor042aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n06x4               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nona22aa1n02x4               g034(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n130));
  norb02aa1n02x5               g035(.a(new_n103), .b(new_n99), .out0(new_n131));
  aoi013aa1n03x5               g036(.a(new_n130), .b(new_n131), .c(new_n106), .d(new_n105), .o1(new_n132));
  norb03aa1n12x5               g037(.a(new_n111), .b(new_n109), .c(new_n110), .out0(new_n133));
  nona23aa1n09x5               g038(.a(new_n133), .b(new_n114), .c(new_n113), .d(new_n119), .out0(new_n134));
  orn002aa1n02x5               g039(.a(\a[6] ), .b(\b[5] ), .o(new_n135));
  oai112aa1n02x5               g040(.a(new_n135), .b(new_n120), .c(\b[4] ), .d(\a[5] ), .o1(new_n136));
  aoi013aa1n06x4               g041(.a(new_n118), .b(new_n136), .c(new_n133), .d(new_n114), .o1(new_n137));
  oai012aa1n12x5               g042(.a(new_n137), .b(new_n132), .c(new_n134), .o1(new_n138));
  aoai13aa1n03x5               g043(.a(new_n128), .b(new_n97), .c(new_n138), .d(new_n124), .o1(new_n139));
  oai012aa1n12x5               g044(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n140));
  nor042aa1n09x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1d04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n03x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  nanp02aa1n02x5               g049(.a(new_n125), .b(new_n98), .o1(new_n145));
  oaoi03aa1n12x5               g050(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n143), .b(new_n146), .c(new_n145), .d(new_n128), .o1(new_n147));
  nor042aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand22aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  aoib12aa1n02x5               g055(.a(new_n141), .b(new_n149), .c(new_n148), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n141), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n143), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n152), .b(new_n153), .c(new_n139), .d(new_n140), .o1(new_n154));
  aoi022aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n147), .d(new_n151), .o1(\s[12] ));
  nona23aa1d18x5               g060(.a(new_n149), .b(new_n142), .c(new_n141), .d(new_n148), .out0(new_n156));
  nano22aa1n03x7               g061(.a(new_n156), .b(new_n124), .c(new_n128), .out0(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n123), .c(new_n116), .d(new_n108), .o1(new_n158));
  oaih12aa1n06x5               g063(.a(new_n149), .b(new_n148), .c(new_n141), .o1(new_n159));
  oai012aa1d24x5               g064(.a(new_n159), .b(new_n156), .c(new_n140), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nor042aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n06x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  xobna2aa1n03x5               g069(.a(new_n164), .b(new_n158), .c(new_n161), .out0(\s[13] ));
  nor002aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand42aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  nanp02aa1n02x5               g073(.a(new_n158), .b(new_n161), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n162), .b(new_n168), .c(new_n169), .d(new_n163), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n162), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n164), .c(new_n158), .d(new_n161), .o1(new_n172));
  aoi012aa1n02x5               g077(.a(new_n170), .b(new_n168), .c(new_n172), .o1(\s[14] ));
  tech160nm_fioaoi03aa1n03p5x5 g078(.a(\a[14] ), .b(\b[13] ), .c(new_n171), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nano23aa1n03x5               g080(.a(new_n162), .b(new_n166), .c(new_n167), .d(new_n163), .out0(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n160), .c(new_n138), .d(new_n157), .o1(new_n177));
  nor002aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand02aa1n12x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n177), .c(new_n175), .out0(\s[15] ));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n174), .c(new_n169), .d(new_n176), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[16] ), .b(\b[15] ), .out0(new_n183));
  norp02aa1n02x5               g088(.a(new_n183), .b(new_n178), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n178), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n179), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n177), .d(new_n175), .o1(new_n187));
  aoi022aa1n02x5               g092(.a(new_n187), .b(new_n183), .c(new_n182), .d(new_n184), .o1(\s[16] ));
  aoi012aa1n06x5               g093(.a(new_n123), .b(new_n116), .c(new_n108), .o1(new_n189));
  and002aa1n12x5               g094(.a(\b[15] ), .b(\a[16] ), .o(new_n190));
  inv000aa1d42x5               g095(.a(new_n190), .o1(new_n191));
  tech160nm_fiaoi012aa1n05x5   g096(.a(new_n178), .b(\a[14] ), .c(\b[13] ), .o1(new_n192));
  tech160nm_fioai012aa1n04x5   g097(.a(new_n179), .b(\b[15] ), .c(\a[16] ), .o1(new_n193));
  nanb03aa1n02x5               g098(.a(new_n193), .b(new_n191), .c(new_n192), .out0(new_n194));
  nano32aa1n03x7               g099(.a(new_n194), .b(new_n168), .c(new_n163), .d(new_n171), .out0(new_n195));
  nand42aa1n02x5               g100(.a(new_n195), .b(new_n157), .o1(new_n196));
  oai022aa1n02x5               g101(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n197));
  aob012aa1n02x5               g102(.a(new_n197), .b(\b[15] ), .c(\a[16] ), .out0(new_n198));
  aoai13aa1n02x5               g103(.a(new_n198), .b(new_n194), .c(new_n171), .d(new_n168), .o1(new_n199));
  aoi012aa1n06x5               g104(.a(new_n199), .b(new_n195), .c(new_n160), .o1(new_n200));
  oai012aa1n12x5               g105(.a(new_n200), .b(new_n189), .c(new_n196), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(\b[16] ), .b(new_n203), .out0(new_n204));
  nano23aa1n06x5               g109(.a(new_n141), .b(new_n148), .c(new_n149), .d(new_n142), .out0(new_n205));
  oai012aa1n02x5               g110(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .o1(new_n206));
  nor043aa1n03x5               g111(.a(new_n193), .b(new_n206), .c(new_n190), .o1(new_n207));
  nand22aa1n03x5               g112(.a(new_n207), .b(new_n176), .o1(new_n208));
  nano32aa1n03x7               g113(.a(new_n208), .b(new_n205), .c(new_n128), .d(new_n124), .out0(new_n209));
  nand42aa1n02x5               g114(.a(new_n205), .b(new_n146), .o1(new_n210));
  nona22aa1n02x4               g115(.a(new_n167), .b(new_n166), .c(new_n162), .out0(new_n211));
  aoi022aa1n02x7               g116(.a(new_n207), .b(new_n211), .c(new_n191), .d(new_n197), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n208), .c(new_n210), .d(new_n159), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[17] ), .b(\b[16] ), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n213), .c(new_n138), .d(new_n209), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n215), .c(new_n204), .out0(\s[18] ));
  inv040aa1d32x5               g122(.a(\a[18] ), .o1(new_n218));
  xroi22aa1d04x5               g123(.a(new_n203), .b(\b[16] ), .c(new_n218), .d(\b[17] ), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n213), .c(new_n138), .d(new_n209), .o1(new_n220));
  oaoi03aa1n02x5               g125(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  nor002aa1d32x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nand42aa1d28x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n220), .c(new_n222), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g132(.a(new_n225), .b(new_n221), .c(new_n201), .d(new_n219), .o1(new_n228));
  nor042aa1n02x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  nand42aa1n08x5               g134(.a(\b[19] ), .b(\a[20] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\a[19] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\b[18] ), .o1(new_n233));
  aboi22aa1n03x5               g138(.a(new_n229), .b(new_n230), .c(new_n232), .d(new_n233), .out0(new_n234));
  inv040aa1n08x5               g139(.a(new_n223), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n224), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n235), .b(new_n236), .c(new_n220), .d(new_n222), .o1(new_n237));
  aoi022aa1n03x5               g142(.a(new_n237), .b(new_n231), .c(new_n228), .d(new_n234), .o1(\s[20] ));
  nand42aa1n10x5               g143(.a(\b[17] ), .b(\a[18] ), .o1(new_n239));
  oai012aa1n02x5               g144(.a(new_n224), .b(\b[19] ), .c(\a[20] ), .o1(new_n240));
  nano23aa1n06x5               g145(.a(new_n240), .b(new_n223), .c(new_n239), .d(new_n230), .out0(new_n241));
  and003aa1n02x5               g146(.a(new_n241), .b(new_n216), .c(new_n214), .o(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n213), .c(new_n138), .d(new_n209), .o1(new_n243));
  inv030aa1d32x5               g148(.a(\b[17] ), .o1(new_n244));
  nand42aa1n06x5               g149(.a(new_n244), .b(new_n218), .o1(new_n245));
  oai112aa1n06x5               g150(.a(new_n245), .b(new_n239), .c(\b[16] ), .d(\a[17] ), .o1(new_n246));
  aoi012aa1n06x5               g151(.a(new_n223), .b(\a[18] ), .c(\b[17] ), .o1(new_n247));
  nano22aa1n09x5               g152(.a(new_n229), .b(new_n224), .c(new_n230), .out0(new_n248));
  oaoi03aa1n09x5               g153(.a(\a[20] ), .b(\b[19] ), .c(new_n235), .o1(new_n249));
  aoi013aa1n09x5               g154(.a(new_n249), .b(new_n248), .c(new_n246), .d(new_n247), .o1(new_n250));
  nor002aa1d32x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  nand02aa1d08x5               g156(.a(\b[20] ), .b(\a[21] ), .o1(new_n252));
  norb02aa1d27x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  xnbna2aa1n03x5               g158(.a(new_n253), .b(new_n243), .c(new_n250), .out0(\s[21] ));
  inv000aa1d42x5               g159(.a(new_n250), .o1(new_n255));
  aoai13aa1n02x5               g160(.a(new_n253), .b(new_n255), .c(new_n201), .d(new_n242), .o1(new_n256));
  nor042aa1n06x5               g161(.a(\b[21] ), .b(\a[22] ), .o1(new_n257));
  nand42aa1d28x5               g162(.a(\b[21] ), .b(\a[22] ), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(new_n259));
  aoib12aa1n02x5               g164(.a(new_n251), .b(new_n258), .c(new_n257), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n251), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n253), .o1(new_n262));
  aoai13aa1n02x5               g167(.a(new_n261), .b(new_n262), .c(new_n243), .d(new_n250), .o1(new_n263));
  aoi022aa1n02x5               g168(.a(new_n263), .b(new_n259), .c(new_n256), .d(new_n260), .o1(\s[22] ));
  nona23aa1n02x4               g169(.a(new_n258), .b(new_n252), .c(new_n251), .d(new_n257), .out0(new_n265));
  nano32aa1n02x4               g170(.a(new_n265), .b(new_n241), .c(new_n216), .d(new_n214), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n213), .c(new_n138), .d(new_n209), .o1(new_n267));
  nano23aa1n03x7               g172(.a(new_n251), .b(new_n257), .c(new_n258), .d(new_n252), .out0(new_n268));
  nona22aa1n09x5               g173(.a(new_n258), .b(new_n257), .c(new_n251), .out0(new_n269));
  aboi22aa1n02x7               g174(.a(new_n250), .b(new_n268), .c(new_n269), .d(new_n258), .out0(new_n270));
  inv000aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[23] ), .b(\b[22] ), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n201), .d(new_n266), .o1(new_n273));
  aoi122aa1n02x5               g178(.a(new_n272), .b(new_n258), .c(new_n269), .d(new_n255), .e(new_n268), .o1(new_n274));
  aobi12aa1n03x7               g179(.a(new_n273), .b(new_n274), .c(new_n267), .out0(\s[23] ));
  xorc02aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(\a[23] ), .o1(new_n277));
  inv000aa1d42x5               g182(.a(\b[22] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(new_n278), .b(new_n277), .o1(new_n279));
  norb02aa1n02x5               g184(.a(new_n279), .b(new_n276), .out0(new_n280));
  nand02aa1d28x5               g185(.a(\b[22] ), .b(\a[23] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  aoai13aa1n03x5               g187(.a(new_n279), .b(new_n282), .c(new_n267), .d(new_n270), .o1(new_n283));
  aoi022aa1n03x5               g188(.a(new_n283), .b(new_n276), .c(new_n273), .d(new_n280), .o1(\s[24] ));
  and002aa1n12x5               g189(.a(\b[23] ), .b(\a[24] ), .o(new_n285));
  oai012aa1n04x7               g190(.a(new_n258), .b(\b[22] ), .c(\a[23] ), .o1(new_n286));
  oai012aa1n09x5               g191(.a(new_n281), .b(\b[23] ), .c(\a[24] ), .o1(new_n287));
  nor043aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n285), .o1(new_n288));
  nand02aa1d04x5               g193(.a(new_n288), .b(new_n268), .o1(new_n289));
  nano32aa1n02x4               g194(.a(new_n289), .b(new_n241), .c(new_n216), .d(new_n214), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n213), .c(new_n138), .d(new_n209), .o1(new_n291));
  nanp03aa1n06x5               g196(.a(new_n248), .b(new_n246), .c(new_n247), .o1(new_n292));
  inv000aa1n02x5               g197(.a(new_n249), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n285), .o1(new_n294));
  oai022aa1n02x5               g199(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n295));
  aoi022aa1n12x5               g200(.a(new_n288), .b(new_n269), .c(new_n294), .d(new_n295), .o1(new_n296));
  aoai13aa1n12x5               g201(.a(new_n296), .b(new_n289), .c(new_n292), .d(new_n293), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[25] ), .b(\b[24] ), .out0(new_n299));
  xnbna2aa1n03x5               g204(.a(new_n299), .b(new_n291), .c(new_n298), .out0(\s[25] ));
  aoai13aa1n02x5               g205(.a(new_n299), .b(new_n297), .c(new_n201), .d(new_n290), .o1(new_n301));
  tech160nm_fixorc02aa1n03p5x5 g206(.a(\a[26] ), .b(\b[25] ), .out0(new_n302));
  nor042aa1n06x5               g207(.a(\b[24] ), .b(\a[25] ), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n302), .b(new_n303), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n303), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n299), .o1(new_n306));
  aoai13aa1n02x5               g211(.a(new_n305), .b(new_n306), .c(new_n291), .d(new_n298), .o1(new_n307));
  aoi022aa1n02x5               g212(.a(new_n307), .b(new_n302), .c(new_n301), .d(new_n304), .o1(\s[26] ));
  nand02aa1d06x5               g213(.a(new_n302), .b(new_n299), .o1(new_n309));
  nano23aa1n03x7               g214(.a(new_n309), .b(new_n289), .c(new_n219), .d(new_n241), .out0(new_n310));
  aoai13aa1n06x5               g215(.a(new_n310), .b(new_n213), .c(new_n138), .d(new_n209), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n309), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[26] ), .b(\b[25] ), .c(new_n305), .carry(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  aoi012aa1n12x5               g219(.a(new_n314), .b(new_n297), .c(new_n312), .o1(new_n315));
  nanp02aa1n02x5               g220(.a(new_n311), .b(new_n315), .o1(new_n316));
  xorc02aa1n12x5               g221(.a(\a[27] ), .b(\b[26] ), .out0(new_n317));
  aoi112aa1n02x5               g222(.a(new_n317), .b(new_n314), .c(new_n297), .d(new_n312), .o1(new_n318));
  aoi022aa1n02x5               g223(.a(new_n316), .b(new_n317), .c(new_n318), .d(new_n311), .o1(\s[27] ));
  aoi022aa1n02x5               g224(.a(new_n278), .b(new_n277), .c(\a[22] ), .d(\b[21] ), .o1(new_n320));
  nanb02aa1n02x5               g225(.a(new_n287), .b(new_n294), .out0(new_n321));
  nano32aa1n03x7               g226(.a(new_n321), .b(new_n259), .c(new_n253), .d(new_n320), .out0(new_n322));
  aoai13aa1n06x5               g227(.a(new_n322), .b(new_n249), .c(new_n246), .d(new_n241), .o1(new_n323));
  aoai13aa1n02x5               g228(.a(new_n313), .b(new_n309), .c(new_n323), .d(new_n296), .o1(new_n324));
  aoai13aa1n02x5               g229(.a(new_n317), .b(new_n324), .c(new_n201), .d(new_n310), .o1(new_n325));
  norp02aa1n02x5               g230(.a(\b[27] ), .b(\a[28] ), .o1(new_n326));
  nand42aa1n08x5               g231(.a(\b[27] ), .b(\a[28] ), .o1(new_n327));
  norb02aa1n03x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  nor042aa1n04x5               g233(.a(\b[26] ), .b(\a[27] ), .o1(new_n329));
  aoib12aa1n02x5               g234(.a(new_n329), .b(new_n327), .c(new_n326), .out0(new_n330));
  inv000aa1d42x5               g235(.a(new_n329), .o1(new_n331));
  inv000aa1n02x5               g236(.a(new_n317), .o1(new_n332));
  aoai13aa1n03x5               g237(.a(new_n331), .b(new_n332), .c(new_n311), .d(new_n315), .o1(new_n333));
  aoi022aa1n03x5               g238(.a(new_n333), .b(new_n328), .c(new_n325), .d(new_n330), .o1(\s[28] ));
  and002aa1n02x5               g239(.a(new_n317), .b(new_n328), .o(new_n335));
  aoai13aa1n02x5               g240(.a(new_n335), .b(new_n324), .c(new_n201), .d(new_n310), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n335), .o1(new_n337));
  aoi012aa1n02x5               g242(.a(new_n326), .b(new_n329), .c(new_n327), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n311), .d(new_n315), .o1(new_n339));
  tech160nm_fixorc02aa1n03p5x5 g244(.a(\a[29] ), .b(\b[28] ), .out0(new_n340));
  norb02aa1n02x5               g245(.a(new_n338), .b(new_n340), .out0(new_n341));
  aoi022aa1n02x7               g246(.a(new_n339), .b(new_n340), .c(new_n336), .d(new_n341), .o1(\s[29] ));
  nanp02aa1n02x5               g247(.a(\b[0] ), .b(\a[1] ), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g249(.a(new_n332), .b(new_n340), .c(new_n328), .out0(new_n345));
  aoai13aa1n03x5               g250(.a(new_n345), .b(new_n324), .c(new_n201), .d(new_n310), .o1(new_n346));
  inv000aa1d42x5               g251(.a(new_n345), .o1(new_n347));
  oao003aa1n02x5               g252(.a(\a[29] ), .b(\b[28] ), .c(new_n338), .carry(new_n348));
  aoai13aa1n03x5               g253(.a(new_n348), .b(new_n347), .c(new_n311), .d(new_n315), .o1(new_n349));
  tech160nm_fixorc02aa1n04x5   g254(.a(\a[30] ), .b(\b[29] ), .out0(new_n350));
  norb02aa1n02x5               g255(.a(new_n348), .b(new_n350), .out0(new_n351));
  aoi022aa1n03x5               g256(.a(new_n349), .b(new_n350), .c(new_n346), .d(new_n351), .o1(\s[30] ));
  nano32aa1n02x5               g257(.a(new_n332), .b(new_n350), .c(new_n328), .d(new_n340), .out0(new_n353));
  aoai13aa1n02x5               g258(.a(new_n353), .b(new_n324), .c(new_n201), .d(new_n310), .o1(new_n354));
  xorc02aa1n02x5               g259(.a(\a[31] ), .b(\b[30] ), .out0(new_n355));
  oao003aa1n02x5               g260(.a(\a[30] ), .b(\b[29] ), .c(new_n348), .carry(new_n356));
  norb02aa1n02x5               g261(.a(new_n356), .b(new_n355), .out0(new_n357));
  inv000aa1d42x5               g262(.a(new_n353), .o1(new_n358));
  aoai13aa1n03x5               g263(.a(new_n356), .b(new_n358), .c(new_n311), .d(new_n315), .o1(new_n359));
  aoi022aa1n03x5               g264(.a(new_n359), .b(new_n355), .c(new_n354), .d(new_n357), .o1(\s[31] ));
  xnbna2aa1n03x5               g265(.a(new_n104), .b(new_n105), .c(new_n106), .out0(\s[3] ));
  norb02aa1n02x5               g266(.a(new_n100), .b(new_n101), .out0(new_n362));
  aoi013aa1n02x4               g267(.a(new_n99), .b(new_n105), .c(new_n106), .d(new_n103), .o1(new_n363));
  oai012aa1n02x5               g268(.a(new_n108), .b(new_n363), .c(new_n362), .o1(\s[4] ));
  xorc02aa1n02x5               g269(.a(\a[5] ), .b(\b[4] ), .out0(new_n365));
  xobna2aa1n03x5               g270(.a(new_n365), .b(new_n108), .c(new_n100), .out0(\s[5] ));
  orn002aa1n02x5               g271(.a(\a[5] ), .b(\b[4] ), .o(new_n367));
  nanp03aa1n02x5               g272(.a(new_n108), .b(new_n100), .c(new_n365), .o1(new_n368));
  xorc02aa1n02x5               g273(.a(\a[6] ), .b(\b[5] ), .out0(new_n369));
  nanp02aa1n03x5               g274(.a(new_n368), .b(new_n121), .o1(new_n370));
  aoai13aa1n02x5               g275(.a(new_n370), .b(new_n369), .c(new_n367), .d(new_n368), .o1(\s[6] ));
  nanp02aa1n02x5               g276(.a(\b[6] ), .b(\a[7] ), .o1(new_n372));
  aoi022aa1n02x5               g277(.a(new_n370), .b(new_n120), .c(new_n117), .d(new_n372), .o1(new_n373));
  aoi013aa1n02x4               g278(.a(new_n373), .b(new_n370), .c(new_n114), .d(new_n117), .o1(\s[7] ));
  norb02aa1n02x5               g279(.a(new_n111), .b(new_n110), .out0(new_n375));
  nanp03aa1n02x5               g280(.a(new_n370), .b(new_n117), .c(new_n114), .o1(new_n376));
  xnbna2aa1n03x5               g281(.a(new_n375), .b(new_n376), .c(new_n117), .out0(\s[8] ));
  aoi112aa1n02x5               g282(.a(new_n123), .b(new_n124), .c(new_n116), .d(new_n108), .o1(new_n378));
  aoi012aa1n02x5               g283(.a(new_n378), .b(new_n138), .c(new_n124), .o1(\s[9] ));
endmodule


