// Benchmark "adder" written by ABC on Thu Jul 18 00:52:31 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n264, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n303, new_n304, new_n305, new_n306, new_n307, new_n308, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n319, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n329, new_n330, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n354, new_n356, new_n357, new_n359, new_n361,
    new_n362, new_n363, new_n365, new_n366, new_n367, new_n368, new_n370,
    new_n372;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n03x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  inv030aa1n04x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n06x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nor042aa1n04x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1d04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor042aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nand23aa1n06x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  tech160nm_fiaoi012aa1n04x5   g013(.a(new_n102), .b(new_n105), .c(new_n103), .o1(new_n109));
  nor002aa1n04x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand02aa1d08x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor042aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand02aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nano23aa1n03x7               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  nor022aa1n16x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand02aa1d24x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor002aa1d24x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand02aa1d06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nano23aa1n02x4               g023(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n119));
  nand02aa1n02x5               g024(.a(new_n119), .b(new_n114), .o1(new_n120));
  norb02aa1n12x5               g025(.a(new_n116), .b(new_n115), .out0(new_n121));
  nano22aa1n09x5               g026(.a(new_n117), .b(new_n111), .c(new_n118), .out0(new_n122));
  oai022aa1n03x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  inv040aa1n02x5               g028(.a(new_n117), .o1(new_n124));
  oaoi03aa1n12x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  aoi013aa1n06x4               g030(.a(new_n125), .b(new_n122), .c(new_n123), .d(new_n121), .o1(new_n126));
  aoai13aa1n12x5               g031(.a(new_n126), .b(new_n120), .c(new_n108), .d(new_n109), .o1(new_n127));
  nor002aa1n06x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nand42aa1n06x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  nanp02aa1n02x5               g035(.a(new_n127), .b(new_n130), .o1(new_n131));
  nor042aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1n08x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoib12aa1n02x5               g039(.a(new_n128), .b(new_n133), .c(new_n132), .out0(new_n135));
  oai012aa1n02x5               g040(.a(new_n131), .b(\b[8] ), .c(\a[9] ), .o1(new_n136));
  aoi022aa1n02x5               g041(.a(new_n136), .b(new_n134), .c(new_n131), .d(new_n135), .o1(\s[10] ));
  nano23aa1d15x5               g042(.a(new_n128), .b(new_n132), .c(new_n133), .d(new_n129), .out0(new_n138));
  nanp02aa1n06x5               g043(.a(new_n127), .b(new_n138), .o1(new_n139));
  oai012aa1n02x5               g044(.a(new_n133), .b(new_n132), .c(new_n128), .o1(new_n140));
  xorc02aa1n12x5               g045(.a(\a[11] ), .b(\b[10] ), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  aob012aa1n03x5               g047(.a(new_n141), .b(new_n139), .c(new_n140), .out0(new_n143));
  nor022aa1n08x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  and002aa1n12x5               g049(.a(\b[11] ), .b(\a[12] ), .o(new_n145));
  nor002aa1n08x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  nor022aa1n06x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  oab012aa1n02x4               g052(.a(new_n147), .b(new_n145), .c(new_n144), .out0(new_n148));
  oaih12aa1n02x5               g053(.a(new_n143), .b(\b[10] ), .c(\a[11] ), .o1(new_n149));
  aoi022aa1n03x5               g054(.a(new_n149), .b(new_n146), .c(new_n143), .d(new_n148), .o1(\s[12] ));
  nand23aa1d12x5               g055(.a(new_n138), .b(new_n141), .c(new_n146), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  aoi112aa1n03x5               g057(.a(new_n145), .b(new_n144), .c(\a[11] ), .d(\b[10] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n147), .b(\a[10] ), .c(\b[9] ), .o1(new_n154));
  oai012aa1n02x5               g059(.a(new_n154), .b(new_n132), .c(new_n128), .o1(new_n155));
  aoib12aa1n09x5               g060(.a(new_n144), .b(new_n147), .c(new_n145), .out0(new_n156));
  oaib12aa1n06x5               g061(.a(new_n156), .b(new_n155), .c(new_n153), .out0(new_n157));
  nor022aa1n08x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand22aa1n04x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n157), .c(new_n127), .d(new_n152), .o1(new_n161));
  oai112aa1n03x5               g066(.a(new_n153), .b(new_n154), .c(new_n132), .d(new_n128), .o1(new_n162));
  nanb03aa1n02x5               g067(.a(new_n160), .b(new_n162), .c(new_n156), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n127), .c(new_n152), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n161), .b(new_n164), .out0(\s[13] ));
  orn002aa1n02x5               g070(.a(\a[13] ), .b(\b[12] ), .o(new_n166));
  nor002aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand22aa1n09x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n161), .c(new_n166), .out0(\s[14] ));
  nona23aa1n02x4               g075(.a(new_n168), .b(new_n159), .c(new_n158), .d(new_n167), .out0(new_n171));
  nona22aa1n06x5               g076(.a(new_n127), .b(new_n151), .c(new_n171), .out0(new_n172));
  nano23aa1n06x5               g077(.a(new_n158), .b(new_n167), .c(new_n168), .d(new_n159), .out0(new_n173));
  oai022aa1n02x5               g078(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n174));
  aoi022aa1n02x5               g079(.a(new_n157), .b(new_n173), .c(new_n168), .d(new_n174), .o1(new_n175));
  nor042aa1n09x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand22aa1n06x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1d27x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aob012aa1n03x5               g083(.a(new_n178), .b(new_n172), .c(new_n175), .out0(new_n179));
  aoi122aa1n02x5               g084(.a(new_n178), .b(new_n168), .c(new_n174), .d(new_n157), .e(new_n173), .o1(new_n180));
  aobi12aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n172), .out0(\s[15] ));
  xorc02aa1n02x5               g086(.a(\a[16] ), .b(\b[15] ), .out0(new_n182));
  inv000aa1d42x5               g087(.a(\a[16] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\b[15] ), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n184), .b(new_n183), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[15] ), .b(\a[16] ), .o1(new_n186));
  aoi012aa1n02x5               g091(.a(new_n176), .b(new_n185), .c(new_n186), .o1(new_n187));
  inv000aa1n04x5               g092(.a(new_n176), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n178), .o1(new_n189));
  aoai13aa1n02x5               g094(.a(new_n188), .b(new_n189), .c(new_n172), .d(new_n175), .o1(new_n190));
  aoi022aa1n02x5               g095(.a(new_n190), .b(new_n182), .c(new_n179), .d(new_n187), .o1(\s[16] ));
  aoi012aa1n02x5               g096(.a(new_n97), .b(new_n99), .c(new_n100), .o1(new_n192));
  nona23aa1n03x5               g097(.a(new_n106), .b(new_n103), .c(new_n102), .d(new_n105), .out0(new_n193));
  tech160nm_fioai012aa1n03p5x5 g098(.a(new_n109), .b(new_n193), .c(new_n192), .o1(new_n194));
  nanb02aa1n06x5               g099(.a(new_n120), .b(new_n194), .out0(new_n195));
  nano32aa1n03x7               g100(.a(new_n171), .b(new_n186), .c(new_n178), .d(new_n185), .out0(new_n196));
  nanb02aa1n03x5               g101(.a(new_n151), .b(new_n196), .out0(new_n197));
  nanp03aa1n02x5               g102(.a(new_n185), .b(new_n177), .c(new_n186), .o1(new_n198));
  oai112aa1n03x5               g103(.a(new_n188), .b(new_n168), .c(new_n167), .d(new_n158), .o1(new_n199));
  tech160nm_fioaoi03aa1n03p5x5 g104(.a(new_n183), .b(new_n184), .c(new_n176), .o1(new_n200));
  tech160nm_fioai012aa1n04x5   g105(.a(new_n200), .b(new_n199), .c(new_n198), .o1(new_n201));
  aoi012aa1n06x5               g106(.a(new_n201), .b(new_n157), .c(new_n196), .o1(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n197), .c(new_n195), .d(new_n126), .o1(new_n203));
  xorc02aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .out0(new_n204));
  nand23aa1n02x5               g109(.a(new_n173), .b(new_n178), .c(new_n182), .o1(new_n205));
  nor042aa1n06x5               g110(.a(new_n205), .b(new_n151), .o1(new_n206));
  nano32aa1n02x4               g111(.a(new_n198), .b(new_n174), .c(new_n188), .d(new_n168), .out0(new_n207));
  nona22aa1n02x4               g112(.a(new_n200), .b(new_n207), .c(new_n204), .out0(new_n208));
  aoi122aa1n02x5               g113(.a(new_n208), .b(new_n157), .c(new_n196), .d(new_n127), .e(new_n206), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n209), .b(new_n203), .c(new_n204), .o1(\s[17] ));
  inv000aa1d42x5               g115(.a(\a[17] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(\b[16] ), .b(new_n211), .out0(new_n212));
  inv000aa1n02x5               g117(.a(new_n201), .o1(new_n213));
  aoai13aa1n12x5               g118(.a(new_n213), .b(new_n205), .c(new_n162), .d(new_n156), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n204), .b(new_n214), .c(new_n127), .d(new_n206), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[18] ), .b(\b[17] ), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n215), .c(new_n212), .out0(\s[18] ));
  inv000aa1d42x5               g122(.a(\a[18] ), .o1(new_n218));
  xroi22aa1d04x5               g123(.a(new_n211), .b(\b[16] ), .c(new_n218), .d(\b[17] ), .out0(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n214), .c(new_n127), .d(new_n206), .o1(new_n220));
  oaoi03aa1n02x5               g125(.a(\a[18] ), .b(\b[17] ), .c(new_n212), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  nor002aa1d32x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  tech160nm_finand02aa1n03p5x5 g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n220), .c(new_n222), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g132(.a(new_n225), .b(new_n221), .c(new_n203), .d(new_n219), .o1(new_n228));
  nor042aa1n04x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  nand42aa1n08x5               g134(.a(\b[19] ), .b(\a[20] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\a[19] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\b[18] ), .o1(new_n233));
  aboi22aa1n03x5               g138(.a(new_n229), .b(new_n230), .c(new_n232), .d(new_n233), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n223), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n225), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n235), .b(new_n236), .c(new_n220), .d(new_n222), .o1(new_n237));
  aoi022aa1n03x5               g142(.a(new_n237), .b(new_n231), .c(new_n228), .d(new_n234), .o1(\s[20] ));
  nano23aa1n03x5               g143(.a(new_n223), .b(new_n229), .c(new_n230), .d(new_n224), .out0(new_n239));
  and003aa1n02x5               g144(.a(new_n239), .b(new_n216), .c(new_n204), .o(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n214), .c(new_n127), .d(new_n206), .o1(new_n241));
  oai022aa1d24x5               g146(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n242));
  nano22aa1n03x7               g147(.a(new_n229), .b(new_n224), .c(new_n230), .out0(new_n243));
  aoi012aa1d18x5               g148(.a(new_n223), .b(\a[18] ), .c(\b[17] ), .o1(new_n244));
  nanp03aa1d12x5               g149(.a(new_n243), .b(new_n242), .c(new_n244), .o1(new_n245));
  aoi012aa1n12x5               g150(.a(new_n229), .b(new_n223), .c(new_n230), .o1(new_n246));
  nand22aa1n12x5               g151(.a(new_n245), .b(new_n246), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(new_n241), .b(new_n248), .o1(new_n249));
  nor042aa1d18x5               g154(.a(\b[20] ), .b(\a[21] ), .o1(new_n250));
  nand42aa1d28x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  nanb03aa1n02x5               g157(.a(new_n229), .b(new_n230), .c(new_n224), .out0(new_n253));
  nano22aa1n02x4               g158(.a(new_n253), .b(new_n244), .c(new_n242), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n252), .o1(new_n255));
  nano22aa1n02x4               g160(.a(new_n254), .b(new_n246), .c(new_n255), .out0(new_n256));
  aoi022aa1n02x5               g161(.a(new_n249), .b(new_n252), .c(new_n241), .d(new_n256), .o1(\s[21] ));
  aoai13aa1n03x5               g162(.a(new_n252), .b(new_n247), .c(new_n203), .d(new_n240), .o1(new_n258));
  nor042aa1n04x5               g163(.a(\b[21] ), .b(\a[22] ), .o1(new_n259));
  nand42aa1d28x5               g164(.a(\b[21] ), .b(\a[22] ), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n260), .b(new_n259), .out0(new_n261));
  aoib12aa1n02x5               g166(.a(new_n250), .b(new_n260), .c(new_n259), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n250), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n255), .c(new_n241), .d(new_n248), .o1(new_n264));
  aoi022aa1n02x7               g169(.a(new_n264), .b(new_n261), .c(new_n258), .d(new_n262), .o1(\s[22] ));
  nano23aa1d15x5               g170(.a(new_n250), .b(new_n259), .c(new_n260), .d(new_n251), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  nano32aa1n02x4               g172(.a(new_n267), .b(new_n239), .c(new_n216), .d(new_n204), .out0(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n214), .c(new_n127), .d(new_n206), .o1(new_n269));
  oaih22aa1d12x5               g174(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n270));
  aoi022aa1n12x5               g175(.a(new_n247), .b(new_n266), .c(new_n260), .d(new_n270), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n269), .b(new_n271), .o1(new_n272));
  nor042aa1n06x5               g177(.a(\b[22] ), .b(\a[23] ), .o1(new_n273));
  nand22aa1n03x5               g178(.a(\b[22] ), .b(\a[23] ), .o1(new_n274));
  norb02aa1n12x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  aoi122aa1n02x5               g180(.a(new_n275), .b(new_n260), .c(new_n270), .d(new_n247), .e(new_n266), .o1(new_n276));
  aoi022aa1n02x5               g181(.a(new_n272), .b(new_n275), .c(new_n269), .d(new_n276), .o1(\s[23] ));
  inv000aa1n02x5               g182(.a(new_n271), .o1(new_n278));
  aoai13aa1n02x5               g183(.a(new_n275), .b(new_n278), .c(new_n203), .d(new_n268), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[24] ), .b(\b[23] ), .out0(new_n280));
  orn002aa1n03x5               g185(.a(\a[24] ), .b(\b[23] ), .o(new_n281));
  nanp02aa1n02x5               g186(.a(\b[23] ), .b(\a[24] ), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n273), .b(new_n281), .c(new_n282), .o1(new_n283));
  inv030aa1n03x5               g188(.a(new_n273), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n275), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n284), .b(new_n285), .c(new_n269), .d(new_n271), .o1(new_n286));
  aoi022aa1n03x5               g191(.a(new_n286), .b(new_n280), .c(new_n279), .d(new_n283), .o1(\s[24] ));
  nand23aa1d12x5               g192(.a(new_n266), .b(new_n275), .c(new_n280), .o1(new_n288));
  nano32aa1n02x4               g193(.a(new_n288), .b(new_n239), .c(new_n216), .d(new_n204), .out0(new_n289));
  aoai13aa1n04x5               g194(.a(new_n289), .b(new_n214), .c(new_n127), .d(new_n206), .o1(new_n290));
  nanp03aa1n02x5               g195(.a(new_n281), .b(new_n274), .c(new_n282), .o1(new_n291));
  nano32aa1n03x5               g196(.a(new_n291), .b(new_n270), .c(new_n284), .d(new_n260), .out0(new_n292));
  aob012aa1n02x5               g197(.a(new_n281), .b(new_n273), .c(new_n282), .out0(new_n293));
  nor042aa1n04x5               g198(.a(new_n292), .b(new_n293), .o1(new_n294));
  aoai13aa1n12x5               g199(.a(new_n294), .b(new_n288), .c(new_n245), .d(new_n246), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n290), .b(new_n296), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[25] ), .b(\b[24] ), .out0(new_n298));
  nanb02aa1n06x5               g203(.a(new_n288), .b(new_n247), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n298), .o1(new_n300));
  nano23aa1n02x4               g205(.a(new_n293), .b(new_n292), .c(new_n299), .d(new_n300), .out0(new_n301));
  aoi022aa1n02x5               g206(.a(new_n297), .b(new_n298), .c(new_n290), .d(new_n301), .o1(\s[25] ));
  aoai13aa1n03x5               g207(.a(new_n298), .b(new_n295), .c(new_n203), .d(new_n289), .o1(new_n303));
  tech160nm_fixorc02aa1n02p5x5 g208(.a(\a[26] ), .b(\b[25] ), .out0(new_n304));
  nor042aa1n03x5               g209(.a(\b[24] ), .b(\a[25] ), .o1(new_n305));
  norp02aa1n02x5               g210(.a(new_n304), .b(new_n305), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n305), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n300), .c(new_n290), .d(new_n296), .o1(new_n308));
  aoi022aa1n03x5               g213(.a(new_n308), .b(new_n304), .c(new_n303), .d(new_n306), .o1(\s[26] ));
  nand22aa1n06x5               g214(.a(new_n304), .b(new_n298), .o1(new_n310));
  nano23aa1n06x5               g215(.a(new_n310), .b(new_n288), .c(new_n219), .d(new_n239), .out0(new_n311));
  aoai13aa1n09x5               g216(.a(new_n311), .b(new_n214), .c(new_n127), .d(new_n206), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n310), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[26] ), .b(\b[25] ), .c(new_n307), .carry(new_n314));
  aobi12aa1n12x5               g219(.a(new_n314), .b(new_n295), .c(new_n313), .out0(new_n315));
  nanp02aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(new_n316));
  xorc02aa1n12x5               g221(.a(\a[27] ), .b(\b[26] ), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  oai112aa1n02x5               g223(.a(new_n314), .b(new_n318), .c(new_n296), .d(new_n310), .o1(new_n319));
  aboi22aa1n03x5               g224(.a(new_n319), .b(new_n312), .c(new_n316), .d(new_n317), .out0(\s[27] ));
  aoai13aa1n12x5               g225(.a(new_n314), .b(new_n310), .c(new_n299), .d(new_n294), .o1(new_n321));
  aoai13aa1n06x5               g226(.a(new_n317), .b(new_n321), .c(new_n203), .d(new_n311), .o1(new_n322));
  tech160nm_fixorc02aa1n02p5x5 g227(.a(\a[28] ), .b(\b[27] ), .out0(new_n323));
  norp02aa1n02x5               g228(.a(\b[26] ), .b(\a[27] ), .o1(new_n324));
  norp02aa1n02x5               g229(.a(new_n323), .b(new_n324), .o1(new_n325));
  inv000aa1n03x5               g230(.a(new_n324), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n318), .c(new_n312), .d(new_n315), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n327), .b(new_n323), .c(new_n322), .d(new_n325), .o1(\s[28] ));
  and002aa1n02x5               g233(.a(new_n323), .b(new_n317), .o(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n321), .c(new_n203), .d(new_n311), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n329), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[28] ), .b(\b[27] ), .c(new_n326), .carry(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n331), .c(new_n312), .d(new_n315), .o1(new_n333));
  xorc02aa1n02x5               g238(.a(\a[29] ), .b(\b[28] ), .out0(new_n334));
  norb02aa1n02x5               g239(.a(new_n332), .b(new_n334), .out0(new_n335));
  aoi022aa1n03x5               g240(.a(new_n333), .b(new_n334), .c(new_n330), .d(new_n335), .o1(\s[29] ));
  xorb03aa1n02x5               g241(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g242(.a(new_n318), .b(new_n323), .c(new_n334), .out0(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n321), .c(new_n203), .d(new_n311), .o1(new_n339));
  inv000aa1d42x5               g244(.a(new_n338), .o1(new_n340));
  oao003aa1n02x5               g245(.a(\a[29] ), .b(\b[28] ), .c(new_n332), .carry(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n340), .c(new_n312), .d(new_n315), .o1(new_n342));
  xorc02aa1n02x5               g247(.a(\a[30] ), .b(\b[29] ), .out0(new_n343));
  norb02aa1n02x5               g248(.a(new_n341), .b(new_n343), .out0(new_n344));
  aoi022aa1n03x5               g249(.a(new_n342), .b(new_n343), .c(new_n339), .d(new_n344), .o1(\s[30] ));
  nano32aa1n02x4               g250(.a(new_n318), .b(new_n343), .c(new_n323), .d(new_n334), .out0(new_n346));
  aoai13aa1n03x5               g251(.a(new_n346), .b(new_n321), .c(new_n203), .d(new_n311), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[31] ), .b(\b[30] ), .out0(new_n348));
  oao003aa1n02x5               g253(.a(\a[30] ), .b(\b[29] ), .c(new_n341), .carry(new_n349));
  norb02aa1n02x5               g254(.a(new_n349), .b(new_n348), .out0(new_n350));
  inv000aa1n02x5               g255(.a(new_n346), .o1(new_n351));
  aoai13aa1n03x5               g256(.a(new_n349), .b(new_n351), .c(new_n312), .d(new_n315), .o1(new_n352));
  aoi022aa1n03x5               g257(.a(new_n352), .b(new_n348), .c(new_n347), .d(new_n350), .o1(\s[31] ));
  aoi112aa1n02x5               g258(.a(new_n107), .b(new_n97), .c(new_n99), .d(new_n100), .o1(new_n354));
  aoi012aa1n02x5               g259(.a(new_n354), .b(new_n101), .c(new_n107), .o1(\s[3] ));
  oaoi03aa1n02x5               g260(.a(\a[3] ), .b(\b[2] ), .c(new_n192), .o1(new_n356));
  aoi112aa1n02x5               g261(.a(new_n105), .b(new_n104), .c(new_n101), .d(new_n106), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n357), .b(new_n104), .c(new_n356), .o1(\s[4] ));
  norb02aa1n02x5               g263(.a(new_n113), .b(new_n112), .out0(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n359), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  norb02aa1n02x5               g265(.a(new_n111), .b(new_n110), .out0(new_n361));
  aoai13aa1n02x5               g266(.a(new_n361), .b(new_n112), .c(new_n194), .d(new_n113), .o1(new_n362));
  aoi112aa1n02x5               g267(.a(new_n112), .b(new_n361), .c(new_n194), .d(new_n359), .o1(new_n363));
  norb02aa1n02x5               g268(.a(new_n362), .b(new_n363), .out0(\s[6] ));
  nanb02aa1n02x5               g269(.a(new_n117), .b(new_n118), .out0(new_n365));
  inv000aa1d42x5               g270(.a(new_n365), .o1(new_n366));
  oai012aa1n02x5               g271(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n367));
  nanp02aa1n02x5               g272(.a(new_n194), .b(new_n114), .o1(new_n368));
  xnbna2aa1n03x5               g273(.a(new_n366), .b(new_n368), .c(new_n367), .out0(\s[7] ));
  aob012aa1n02x5               g274(.a(new_n366), .b(new_n368), .c(new_n367), .out0(new_n370));
  xnbna2aa1n03x5               g275(.a(new_n121), .b(new_n370), .c(new_n124), .out0(\s[8] ));
  aoi113aa1n02x5               g276(.a(new_n130), .b(new_n125), .c(new_n122), .d(new_n123), .e(new_n121), .o1(new_n372));
  aoi022aa1n02x5               g277(.a(new_n127), .b(new_n130), .c(new_n195), .d(new_n372), .o1(\s[9] ));
endmodule


