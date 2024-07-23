// Benchmark "adder" written by ABC on Thu Jul 18 05:40:39 2024

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
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n243, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n264, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n303, new_n304, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n333,
    new_n334, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n352, new_n353, new_n355, new_n358, new_n359, new_n361, new_n362,
    new_n364, new_n366, new_n367;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[2] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[1] ), .o1(new_n98));
  nand22aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n06x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  norp02aa1n12x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand22aa1n09x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n06x5               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n02x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nor002aa1n20x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand42aa1n16x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norb02aa1n02x7               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nor042aa1n04x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand02aa1d24x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  norb02aa1n03x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  nano22aa1n03x7               g023(.a(new_n112), .b(new_n115), .c(new_n118), .out0(new_n119));
  nanp02aa1n02x5               g024(.a(new_n107), .b(new_n119), .o1(new_n120));
  norb02aa1n06x4               g025(.a(new_n109), .b(new_n108), .out0(new_n121));
  nano22aa1n03x7               g026(.a(new_n110), .b(new_n111), .c(new_n114), .out0(new_n122));
  oai022aa1d24x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  inv040aa1n09x5               g028(.a(new_n110), .o1(new_n124));
  oaoi03aa1n09x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  norp02aa1n02x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi113aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n122), .d(new_n123), .e(new_n121), .o1(new_n127));
  nanp02aa1n03x5               g032(.a(new_n120), .b(new_n127), .o1(new_n128));
  and002aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  nand42aa1d28x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  oai012aa1d24x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .o1(new_n131));
  nona22aa1n02x4               g036(.a(new_n128), .b(new_n129), .c(new_n131), .out0(new_n132));
  xnrc02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .out0(new_n133));
  aobi12aa1n02x5               g038(.a(new_n133), .b(new_n128), .c(new_n130), .out0(new_n134));
  norb02aa1n02x5               g039(.a(new_n132), .b(new_n134), .out0(\s[10] ));
  inv000aa1d48x5               g040(.a(\a[10] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(\b[9] ), .o1(new_n137));
  oaoi03aa1n02x5               g042(.a(new_n136), .b(new_n137), .c(new_n126), .o1(new_n138));
  nor042aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand42aa1n08x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n12x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n132), .c(new_n138), .out0(\s[11] ));
  aob012aa1n03x5               g047(.a(new_n141), .b(new_n132), .c(new_n138), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n143), .b(\b[10] ), .c(\a[11] ), .o1(new_n144));
  xorc02aa1n12x5               g049(.a(\a[12] ), .b(\b[11] ), .out0(new_n145));
  norp02aa1n02x5               g050(.a(new_n145), .b(new_n139), .o1(new_n146));
  aoi022aa1n02x5               g051(.a(new_n144), .b(new_n145), .c(new_n143), .d(new_n146), .o1(\s[12] ));
  nanb03aa1n02x5               g052(.a(new_n110), .b(new_n114), .c(new_n111), .out0(new_n148));
  nanb03aa1n02x5               g053(.a(new_n108), .b(new_n123), .c(new_n109), .out0(new_n149));
  oabi12aa1n06x5               g054(.a(new_n125), .b(new_n149), .c(new_n148), .out0(new_n150));
  oai022aa1d18x5               g055(.a(new_n136), .b(new_n137), .c(\b[8] ), .d(\a[9] ), .o1(new_n151));
  nona23aa1d16x5               g056(.a(new_n145), .b(new_n141), .c(new_n151), .d(new_n131), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n150), .c(new_n107), .d(new_n119), .o1(new_n154));
  tech160nm_finand02aa1n03p5x5 g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  aoi112aa1n03x5               g060(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n156));
  aoai13aa1n03x5               g061(.a(new_n140), .b(new_n156), .c(new_n136), .d(new_n137), .o1(new_n157));
  oab012aa1n02x4               g062(.a(new_n139), .b(\a[12] ), .c(\b[11] ), .out0(new_n158));
  nanp02aa1n02x5               g063(.a(new_n157), .b(new_n158), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n159), .b(new_n155), .o1(new_n160));
  nor002aa1n16x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand22aa1n12x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n154), .c(new_n160), .out0(\s[13] ));
  nor042aa1n06x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand42aa1n04x5               g070(.a(new_n154), .b(new_n160), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n161), .o1(new_n167));
  nand02aa1d10x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  oaib12aa1n02x5               g073(.a(new_n167), .b(new_n165), .c(new_n168), .out0(new_n169));
  aoi012aa1n02x5               g074(.a(new_n169), .b(new_n166), .c(new_n163), .o1(new_n170));
  nano23aa1n06x5               g075(.a(new_n161), .b(new_n165), .c(new_n168), .d(new_n162), .out0(new_n171));
  oai022aa1n02x5               g076(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n172));
  aoi022aa1n02x5               g077(.a(new_n166), .b(new_n171), .c(new_n168), .d(new_n172), .o1(new_n173));
  oab012aa1n02x4               g078(.a(new_n170), .b(new_n173), .c(new_n165), .out0(\s[14] ));
  oaoi03aa1n02x5               g079(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n175));
  nor002aa1d24x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand02aa1n06x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n175), .c(new_n166), .d(new_n171), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n175), .c(new_n166), .d(new_n171), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(\s[15] ));
  inv000aa1d42x5               g086(.a(new_n176), .o1(new_n182));
  nor002aa1n10x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand02aa1d12x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  norb03aa1n02x5               g090(.a(new_n184), .b(new_n176), .c(new_n183), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n179), .b(new_n186), .o1(new_n187));
  aoai13aa1n02x5               g092(.a(new_n187), .b(new_n185), .c(new_n182), .d(new_n179), .o1(\s[16] ));
  nano23aa1n06x5               g093(.a(new_n176), .b(new_n183), .c(new_n184), .d(new_n177), .out0(new_n189));
  nano22aa1n12x5               g094(.a(new_n152), .b(new_n171), .c(new_n189), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n150), .c(new_n107), .d(new_n119), .o1(new_n191));
  nano22aa1d15x5               g096(.a(new_n183), .b(new_n177), .c(new_n184), .out0(new_n192));
  tech160nm_fioai012aa1n04x5   g097(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .o1(new_n193));
  oaih12aa1n02x5               g098(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n165), .b(\a[13] ), .c(\b[12] ), .o1(new_n195));
  nona23aa1n03x5               g100(.a(new_n192), .b(new_n195), .c(new_n194), .d(new_n193), .out0(new_n196));
  oab012aa1n03x5               g101(.a(new_n193), .b(new_n161), .c(new_n165), .out0(new_n197));
  oai012aa1n02x5               g102(.a(new_n184), .b(new_n183), .c(new_n176), .o1(new_n198));
  aobi12aa1n06x5               g103(.a(new_n198), .b(new_n197), .c(new_n192), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n196), .c(new_n157), .d(new_n158), .o1(new_n200));
  inv040aa1n08x5               g105(.a(new_n200), .o1(new_n201));
  nanp02aa1n09x5               g106(.a(new_n191), .b(new_n201), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  nand42aa1d28x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoai13aa1n02x5               g110(.a(new_n189), .b(new_n175), .c(new_n166), .d(new_n171), .o1(new_n206));
  oaoi13aa1n02x5               g111(.a(new_n205), .b(new_n184), .c(new_n176), .d(new_n183), .o1(new_n207));
  aoi022aa1n02x5               g112(.a(new_n206), .b(new_n207), .c(new_n205), .d(new_n202), .o1(\s[17] ));
  nor002aa1d32x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nand42aa1d28x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  obai22aa1n02x7               g115(.a(new_n210), .b(new_n209), .c(\a[17] ), .d(\b[16] ), .out0(new_n211));
  aoi012aa1n02x5               g116(.a(new_n211), .b(new_n202), .c(new_n205), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(new_n98), .b(new_n97), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(\b[1] ), .b(\a[2] ), .o1(new_n214));
  aob012aa1n02x5               g119(.a(new_n213), .b(new_n99), .c(new_n214), .out0(new_n215));
  norb02aa1n02x5               g120(.a(new_n102), .b(new_n101), .out0(new_n216));
  norb02aa1n02x5               g121(.a(new_n104), .b(new_n103), .out0(new_n217));
  nanp03aa1n02x5               g122(.a(new_n215), .b(new_n216), .c(new_n217), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(new_n108), .b(new_n109), .out0(new_n219));
  nanb02aa1n12x5               g124(.a(new_n110), .b(new_n111), .out0(new_n220));
  nano23aa1n02x4               g125(.a(new_n113), .b(new_n116), .c(new_n117), .d(new_n114), .out0(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n220), .c(new_n219), .out0(new_n222));
  aoi013aa1n06x4               g127(.a(new_n125), .b(new_n122), .c(new_n123), .d(new_n121), .o1(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n222), .c(new_n218), .d(new_n106), .o1(new_n224));
  nano23aa1d15x5               g129(.a(new_n203), .b(new_n209), .c(new_n210), .d(new_n204), .out0(new_n225));
  aoai13aa1n02x5               g130(.a(new_n225), .b(new_n200), .c(new_n224), .d(new_n190), .o1(new_n226));
  norp02aa1n06x5               g131(.a(new_n209), .b(new_n203), .o1(new_n227));
  norb02aa1n06x4               g132(.a(new_n210), .b(new_n227), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1n02x5               g134(.a(new_n209), .b(new_n226), .c(new_n229), .o1(new_n230));
  norp02aa1n02x5               g135(.a(new_n230), .b(new_n212), .o1(\s[18] ));
  nor042aa1d18x5               g136(.a(\b[18] ), .b(\a[19] ), .o1(new_n232));
  nand22aa1n09x5               g137(.a(\b[18] ), .b(\a[19] ), .o1(new_n233));
  norb02aa1n06x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  xnbna2aa1n03x5               g139(.a(new_n234), .b(new_n226), .c(new_n229), .out0(\s[19] ));
  xnrc02aa1n02x5               g140(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv030aa1n03x5               g141(.a(new_n232), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n234), .b(new_n228), .c(new_n202), .d(new_n225), .o1(new_n238));
  nor002aa1d32x5               g143(.a(\b[19] ), .b(\a[20] ), .o1(new_n239));
  nand02aa1n10x5               g144(.a(\b[19] ), .b(\a[20] ), .o1(new_n240));
  norb02aa1n12x5               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  norp02aa1n02x5               g146(.a(new_n239), .b(new_n232), .o1(new_n242));
  nand43aa1n03x5               g147(.a(new_n238), .b(new_n240), .c(new_n242), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n241), .c(new_n238), .d(new_n237), .o1(\s[20] ));
  nand03aa1n10x5               g149(.a(new_n225), .b(new_n234), .c(new_n241), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  nanb03aa1n03x5               g151(.a(new_n239), .b(new_n240), .c(new_n233), .out0(new_n247));
  oai112aa1n06x5               g152(.a(new_n237), .b(new_n210), .c(new_n209), .d(new_n203), .o1(new_n248));
  oaih12aa1n06x5               g153(.a(new_n240), .b(new_n239), .c(new_n232), .o1(new_n249));
  oai012aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n247), .o1(new_n250));
  tech160nm_fixorc02aa1n02p5x5 g155(.a(\a[21] ), .b(\b[20] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n202), .d(new_n246), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n239), .b(new_n233), .c(new_n240), .out0(new_n253));
  oai012aa1n02x5               g158(.a(new_n210), .b(\b[18] ), .c(\a[19] ), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n253), .b(new_n227), .c(new_n254), .out0(new_n255));
  nano22aa1n02x4               g160(.a(new_n251), .b(new_n255), .c(new_n249), .out0(new_n256));
  aobi12aa1n02x5               g161(.a(new_n256), .b(new_n202), .c(new_n246), .out0(new_n257));
  norb02aa1n03x4               g162(.a(new_n252), .b(new_n257), .out0(\s[21] ));
  inv000aa1d42x5               g163(.a(\a[21] ), .o1(new_n259));
  nanb02aa1n12x5               g164(.a(\b[20] ), .b(new_n259), .out0(new_n260));
  xorc02aa1n02x5               g165(.a(\a[22] ), .b(\b[21] ), .out0(new_n261));
  oai022aa1n02x5               g166(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n262));
  aoi012aa1n02x5               g167(.a(new_n262), .b(\a[22] ), .c(\b[21] ), .o1(new_n263));
  tech160nm_finand02aa1n03p5x5 g168(.a(new_n252), .b(new_n263), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n261), .c(new_n252), .d(new_n260), .o1(\s[22] ));
  nanp02aa1n02x5               g170(.a(new_n261), .b(new_n251), .o1(new_n266));
  nano32aa1n02x4               g171(.a(new_n266), .b(new_n225), .c(new_n234), .d(new_n241), .out0(new_n267));
  oaoi03aa1n12x5               g172(.a(\a[22] ), .b(\b[21] ), .c(new_n260), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n266), .c(new_n255), .d(new_n249), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[23] ), .b(\b[22] ), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n202), .d(new_n267), .o1(new_n272));
  inv000aa1d42x5               g177(.a(\b[21] ), .o1(new_n273));
  xroi22aa1d06x4               g178(.a(new_n273), .b(\a[22] ), .c(new_n259), .d(\b[20] ), .out0(new_n274));
  aoi112aa1n02x5               g179(.a(new_n271), .b(new_n268), .c(new_n250), .d(new_n274), .o1(new_n275));
  aobi12aa1n02x5               g180(.a(new_n275), .b(new_n202), .c(new_n267), .out0(new_n276));
  norb02aa1n03x4               g181(.a(new_n272), .b(new_n276), .out0(\s[23] ));
  nor042aa1n03x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  inv000aa1n03x5               g183(.a(new_n278), .o1(new_n279));
  xorc02aa1n02x5               g184(.a(\a[24] ), .b(\b[23] ), .out0(new_n280));
  and002aa1n02x5               g185(.a(\b[23] ), .b(\a[24] ), .o(new_n281));
  oai022aa1n02x5               g186(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n282));
  nona22aa1n02x5               g187(.a(new_n272), .b(new_n281), .c(new_n282), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n280), .c(new_n272), .d(new_n279), .o1(\s[24] ));
  nanp02aa1n02x5               g189(.a(\b[22] ), .b(\a[23] ), .o1(new_n285));
  tech160nm_fixnrc02aa1n03p5x5 g190(.a(\b[23] ), .b(\a[24] ), .out0(new_n286));
  nano22aa1n06x5               g191(.a(new_n286), .b(new_n279), .c(new_n285), .out0(new_n287));
  nano22aa1n03x7               g192(.a(new_n245), .b(new_n274), .c(new_n287), .out0(new_n288));
  aoai13aa1n04x5               g193(.a(new_n287), .b(new_n268), .c(new_n250), .d(new_n274), .o1(new_n289));
  oaoi03aa1n12x5               g194(.a(\a[24] ), .b(\b[23] ), .c(new_n279), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(new_n292));
  xorc02aa1n12x5               g197(.a(\a[25] ), .b(\b[24] ), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n292), .c(new_n202), .d(new_n288), .o1(new_n294));
  nona22aa1n02x4               g199(.a(new_n289), .b(new_n290), .c(new_n293), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n202), .c(new_n288), .o1(new_n296));
  norb02aa1n02x5               g201(.a(new_n294), .b(new_n296), .out0(\s[25] ));
  nor042aa1n02x5               g202(.a(\b[24] ), .b(\a[25] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  xorc02aa1n12x5               g204(.a(\a[26] ), .b(\b[25] ), .out0(new_n300));
  inv000aa1d42x5               g205(.a(\a[26] ), .o1(new_n301));
  inv000aa1d42x5               g206(.a(\b[25] ), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n298), .b(new_n301), .c(new_n302), .o1(new_n303));
  oai112aa1n03x5               g208(.a(new_n294), .b(new_n303), .c(new_n302), .d(new_n301), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n294), .d(new_n299), .o1(\s[26] ));
  nand42aa1n06x5               g210(.a(new_n300), .b(new_n293), .o1(new_n306));
  nano23aa1d12x5               g211(.a(new_n306), .b(new_n245), .c(new_n274), .d(new_n287), .out0(new_n307));
  aoai13aa1n06x5               g212(.a(new_n307), .b(new_n200), .c(new_n224), .d(new_n190), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n306), .o1(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n290), .c(new_n270), .d(new_n287), .o1(new_n310));
  oaoi03aa1n02x5               g215(.a(new_n301), .b(new_n302), .c(new_n298), .o1(new_n311));
  nand43aa1n02x5               g216(.a(new_n308), .b(new_n310), .c(new_n311), .o1(new_n312));
  xorc02aa1n12x5               g217(.a(\a[27] ), .b(\b[26] ), .out0(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  and003aa1n02x5               g219(.a(new_n310), .b(new_n314), .c(new_n311), .o(new_n315));
  aoi022aa1n02x5               g220(.a(new_n315), .b(new_n308), .c(new_n312), .d(new_n313), .o1(\s[27] ));
  norp02aa1n02x5               g221(.a(\b[26] ), .b(\a[27] ), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  aobi12aa1n06x5               g223(.a(new_n307), .b(new_n191), .c(new_n201), .out0(new_n319));
  aoai13aa1n06x5               g224(.a(new_n311), .b(new_n306), .c(new_n289), .d(new_n291), .o1(new_n320));
  oaih12aa1n02x5               g225(.a(new_n313), .b(new_n320), .c(new_n319), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[28] ), .b(\b[27] ), .out0(new_n322));
  oai022aa1d24x5               g227(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n323));
  aoi012aa1n02x5               g228(.a(new_n323), .b(\a[28] ), .c(\b[27] ), .o1(new_n324));
  tech160nm_finand02aa1n03p5x5 g229(.a(new_n321), .b(new_n324), .o1(new_n325));
  aoai13aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n321), .d(new_n318), .o1(\s[28] ));
  xorc02aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .out0(new_n327));
  and002aa1n02x5               g232(.a(new_n322), .b(new_n313), .o(new_n328));
  oaih12aa1n02x5               g233(.a(new_n328), .b(new_n320), .c(new_n319), .o1(new_n329));
  inv000aa1d42x5               g234(.a(\b[27] ), .o1(new_n330));
  oaib12aa1n09x5               g235(.a(new_n323), .b(new_n330), .c(\a[28] ), .out0(new_n331));
  nand43aa1n03x5               g236(.a(new_n329), .b(new_n331), .c(new_n327), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n331), .o1(new_n333));
  oaoi13aa1n03x5               g238(.a(new_n333), .b(new_n328), .c(new_n320), .d(new_n319), .o1(new_n334));
  oaih12aa1n02x5               g239(.a(new_n332), .b(new_n334), .c(new_n327), .o1(\s[29] ));
  xorb03aa1n02x5               g240(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g241(.a(new_n314), .b(new_n322), .c(new_n327), .out0(new_n337));
  tech160nm_fioaoi03aa1n03p5x5 g242(.a(\a[29] ), .b(\b[28] ), .c(new_n331), .o1(new_n338));
  oaoi13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n320), .d(new_n319), .o1(new_n339));
  xorc02aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .out0(new_n340));
  oaih12aa1n02x5               g245(.a(new_n337), .b(new_n320), .c(new_n319), .o1(new_n341));
  norb02aa1n02x5               g246(.a(new_n340), .b(new_n338), .out0(new_n342));
  tech160nm_finand02aa1n03p5x5 g247(.a(new_n341), .b(new_n342), .o1(new_n343));
  oaih12aa1n02x5               g248(.a(new_n343), .b(new_n339), .c(new_n340), .o1(\s[30] ));
  xnrc02aa1n02x5               g249(.a(\b[30] ), .b(\a[31] ), .out0(new_n345));
  nano32aa1n02x4               g250(.a(new_n314), .b(new_n340), .c(new_n322), .d(new_n327), .out0(new_n346));
  aoi012aa1n02x5               g251(.a(new_n342), .b(\a[30] ), .c(\b[29] ), .o1(new_n347));
  aoai13aa1n03x5               g252(.a(new_n345), .b(new_n347), .c(new_n312), .d(new_n346), .o1(new_n348));
  oaih12aa1n02x5               g253(.a(new_n346), .b(new_n320), .c(new_n319), .o1(new_n349));
  nona22aa1n02x5               g254(.a(new_n349), .b(new_n347), .c(new_n345), .out0(new_n350));
  nanp02aa1n03x5               g255(.a(new_n348), .b(new_n350), .o1(\s[31] ));
  and003aa1n02x5               g256(.a(new_n213), .b(new_n214), .c(new_n99), .o(new_n352));
  nanb03aa1n02x5               g257(.a(new_n103), .b(new_n213), .c(new_n104), .out0(new_n353));
  oai022aa1n02x5               g258(.a(new_n352), .b(new_n353), .c(new_n100), .d(new_n217), .o1(\s[3] ));
  oaoi13aa1n02x5               g259(.a(new_n216), .b(new_n104), .c(new_n352), .d(new_n353), .o1(new_n355));
  aoib12aa1n02x5               g260(.a(new_n355), .b(new_n107), .c(new_n101), .out0(\s[4] ));
  xnbna2aa1n03x5               g261(.a(new_n118), .b(new_n218), .c(new_n106), .out0(\s[5] ));
  aoai13aa1n02x5               g262(.a(new_n115), .b(new_n116), .c(new_n107), .d(new_n117), .o1(new_n358));
  aoi112aa1n02x5               g263(.a(new_n116), .b(new_n115), .c(new_n107), .d(new_n118), .o1(new_n359));
  norb02aa1n02x5               g264(.a(new_n358), .b(new_n359), .out0(\s[6] ));
  inv000aa1d42x5               g265(.a(new_n220), .o1(new_n361));
  inv000aa1d42x5               g266(.a(new_n113), .o1(new_n362));
  xnbna2aa1n03x5               g267(.a(new_n361), .b(new_n358), .c(new_n362), .out0(\s[7] ));
  aob012aa1n02x5               g268(.a(new_n361), .b(new_n358), .c(new_n362), .out0(new_n364));
  xnbna2aa1n03x5               g269(.a(new_n121), .b(new_n364), .c(new_n124), .out0(\s[8] ));
  norb02aa1n02x5               g270(.a(new_n130), .b(new_n126), .out0(new_n366));
  aoi113aa1n02x5               g271(.a(new_n366), .b(new_n125), .c(new_n122), .d(new_n123), .e(new_n121), .o1(new_n367));
  aoi022aa1n02x5               g272(.a(new_n224), .b(new_n366), .c(new_n120), .d(new_n367), .o1(\s[9] ));
endmodule


