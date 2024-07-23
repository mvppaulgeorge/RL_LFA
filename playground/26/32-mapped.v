// Benchmark "adder" written by ABC on Thu Jul 18 01:34:17 2024

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
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n349, new_n350, new_n351,
    new_n352, new_n354, new_n355, new_n357, new_n359, new_n360, new_n361,
    new_n363, new_n364, new_n365, new_n366, new_n368, new_n369, new_n371;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand02aa1d28x5               g001(.a(\b[0] ), .b(\a[1] ), .o1(new_n97));
  inv040aa1n03x5               g002(.a(new_n97), .o1(new_n98));
  tech160nm_fioaoi03aa1n04x5   g003(.a(\a[2] ), .b(\b[1] ), .c(new_n98), .o1(new_n99));
  nor002aa1d24x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nand02aa1d16x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nor002aa1d32x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nano23aa1n02x5               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  nand42aa1n02x5               g009(.a(new_n104), .b(new_n99), .o1(new_n105));
  aoi012aa1n09x5               g010(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n106));
  xnrc02aa1n12x5               g011(.a(\b[5] ), .b(\a[6] ), .out0(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .out0(new_n108));
  nor042aa1n09x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand42aa1d28x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  norp02aa1n24x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n20x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n09x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  nona22aa1n03x5               g018(.a(new_n113), .b(new_n108), .c(new_n107), .out0(new_n114));
  nor042aa1d18x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  inv040aa1n02x5               g020(.a(new_n115), .o1(new_n116));
  oaoi03aa1n09x5               g021(.a(\a[6] ), .b(\b[5] ), .c(new_n116), .o1(new_n117));
  aoi012aa1n06x5               g022(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n118));
  aobi12aa1n06x5               g023(.a(new_n118), .b(new_n113), .c(new_n117), .out0(new_n119));
  aoai13aa1n12x5               g024(.a(new_n119), .b(new_n114), .c(new_n105), .d(new_n106), .o1(new_n120));
  nor042aa1n04x5               g025(.a(\b[8] ), .b(\a[9] ), .o1(new_n121));
  nand42aa1n04x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  norb02aa1n02x5               g027(.a(new_n122), .b(new_n121), .out0(new_n123));
  norp02aa1n04x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  nand02aa1d08x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(new_n124), .b(new_n125), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n121), .c(new_n120), .d(new_n122), .o1(new_n127));
  nona22aa1n02x4               g032(.a(new_n125), .b(new_n121), .c(new_n124), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n127), .b(new_n128), .c(new_n123), .d(new_n120), .o1(\s[10] ));
  inv000aa1d42x5               g034(.a(\a[2] ), .o1(new_n130));
  inv000aa1d42x5               g035(.a(\b[1] ), .o1(new_n131));
  tech160nm_fioaoi03aa1n03p5x5 g036(.a(new_n130), .b(new_n131), .c(new_n97), .o1(new_n132));
  nona23aa1n09x5               g037(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n133));
  oai012aa1n12x5               g038(.a(new_n106), .b(new_n133), .c(new_n132), .o1(new_n134));
  nona23aa1n03x5               g039(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n135));
  nor043aa1n03x5               g040(.a(new_n135), .b(new_n108), .c(new_n107), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(new_n113), .b(new_n117), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n137), .b(new_n118), .o1(new_n138));
  nano23aa1n06x5               g043(.a(new_n124), .b(new_n121), .c(new_n122), .d(new_n125), .out0(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n134), .d(new_n136), .o1(new_n140));
  oai012aa1n02x5               g045(.a(new_n125), .b(new_n121), .c(new_n124), .o1(new_n141));
  xorc02aa1n06x5               g046(.a(\a[11] ), .b(\b[10] ), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n140), .c(new_n141), .out0(\s[11] ));
  nor002aa1n12x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  aob012aa1n02x5               g050(.a(new_n142), .b(new_n140), .c(new_n141), .out0(new_n146));
  nor042aa1n02x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1n06x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n09x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  nona23aa1n02x4               g054(.a(new_n146), .b(new_n148), .c(new_n147), .d(new_n144), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n145), .d(new_n146), .o1(\s[12] ));
  nanp03aa1n02x5               g056(.a(new_n139), .b(new_n142), .c(new_n149), .o1(new_n152));
  nanb02aa1n02x5               g057(.a(new_n152), .b(new_n120), .out0(new_n153));
  nanp02aa1n03x5               g058(.a(new_n134), .b(new_n136), .o1(new_n154));
  inv000aa1n02x5               g059(.a(new_n147), .o1(new_n155));
  oai022aa1d18x5               g060(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n156));
  aoi022aa1d24x5               g061(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n148), .b(new_n144), .c(new_n156), .d(new_n157), .o1(new_n158));
  nand22aa1n02x5               g063(.a(new_n158), .b(new_n155), .o1(new_n159));
  inv000aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n04x5               g065(.a(new_n160), .b(new_n152), .c(new_n154), .d(new_n119), .o1(new_n161));
  nor042aa1d18x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1d28x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  nano22aa1n02x4               g069(.a(new_n164), .b(new_n158), .c(new_n155), .out0(new_n165));
  aoi022aa1n02x5               g070(.a(new_n161), .b(new_n164), .c(new_n153), .d(new_n165), .o1(\s[13] ));
  inv000aa1d42x5               g071(.a(new_n162), .o1(new_n167));
  nand42aa1n03x5               g072(.a(new_n161), .b(new_n164), .o1(new_n168));
  nor042aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  oaih22aa1d12x5               g076(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n172));
  nanb03aa1n02x5               g077(.a(new_n172), .b(new_n168), .c(new_n170), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n171), .c(new_n167), .d(new_n168), .o1(\s[14] ));
  nano23aa1n09x5               g079(.a(new_n162), .b(new_n169), .c(new_n170), .d(new_n163), .out0(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n176));
  nor042aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nand02aa1n06x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n176), .c(new_n161), .d(new_n175), .o1(new_n180));
  aoi112aa1n02x5               g085(.a(new_n179), .b(new_n176), .c(new_n161), .d(new_n175), .o1(new_n181));
  norb02aa1n02x5               g086(.a(new_n180), .b(new_n181), .out0(\s[15] ));
  inv040aa1n02x5               g087(.a(new_n177), .o1(new_n183));
  nor042aa1d18x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nand02aa1d28x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  inv000aa1d42x5               g091(.a(new_n185), .o1(new_n187));
  oai022aa1n02x5               g092(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n188));
  nona22aa1n03x5               g093(.a(new_n180), .b(new_n187), .c(new_n188), .out0(new_n189));
  aoai13aa1n03x5               g094(.a(new_n189), .b(new_n186), .c(new_n183), .d(new_n180), .o1(\s[16] ));
  nano23aa1n09x5               g095(.a(new_n177), .b(new_n184), .c(new_n185), .d(new_n178), .out0(new_n191));
  nand22aa1n03x5               g096(.a(new_n191), .b(new_n175), .o1(new_n192));
  nano32aa1n03x7               g097(.a(new_n192), .b(new_n139), .c(new_n142), .d(new_n149), .out0(new_n193));
  aoai13aa1n06x5               g098(.a(new_n193), .b(new_n138), .c(new_n134), .d(new_n136), .o1(new_n194));
  inv000aa1d42x5               g099(.a(new_n184), .o1(new_n195));
  nanp03aa1n02x5               g100(.a(new_n172), .b(new_n170), .c(new_n178), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n195), .b(new_n187), .c(new_n196), .d(new_n183), .o1(new_n197));
  aoib12aa1n06x5               g102(.a(new_n197), .b(new_n159), .c(new_n192), .out0(new_n198));
  nanp02aa1n09x5               g103(.a(new_n194), .b(new_n198), .o1(new_n199));
  nor002aa1d32x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  nand42aa1n20x5               g105(.a(\b[16] ), .b(\a[17] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n196), .b(new_n183), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n184), .c(new_n203), .d(new_n185), .o1(new_n204));
  oa0012aa1n02x5               g109(.a(new_n204), .b(new_n160), .c(new_n192), .o(new_n205));
  aoi022aa1n02x5               g110(.a(new_n199), .b(new_n202), .c(new_n205), .d(new_n194), .o1(\s[17] ));
  inv000aa1d42x5               g111(.a(new_n200), .o1(new_n207));
  aoi012aa1n02x5               g112(.a(new_n184), .b(new_n203), .c(new_n185), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n192), .c(new_n155), .d(new_n158), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n202), .b(new_n209), .c(new_n120), .d(new_n193), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nand42aa1n20x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  nona23aa1n02x4               g118(.a(new_n210), .b(new_n212), .c(new_n211), .d(new_n200), .out0(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n213), .c(new_n207), .d(new_n210), .o1(\s[18] ));
  nano23aa1d15x5               g120(.a(new_n200), .b(new_n211), .c(new_n212), .d(new_n201), .out0(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n207), .o1(new_n217));
  nor042aa1n06x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand02aa1n03x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  norb02aa1n09x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n217), .c(new_n199), .d(new_n216), .o1(new_n221));
  aoi112aa1n02x5               g126(.a(new_n220), .b(new_n217), .c(new_n199), .d(new_n216), .o1(new_n222));
  norb02aa1n03x4               g127(.a(new_n221), .b(new_n222), .out0(\s[19] ));
  xnrc02aa1n02x5               g128(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv020aa1n02x5               g129(.a(new_n218), .o1(new_n225));
  nor042aa1n04x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nand02aa1d08x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  norb02aa1n06x4               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  norb03aa1n02x5               g133(.a(new_n227), .b(new_n218), .c(new_n226), .out0(new_n229));
  nanp02aa1n03x5               g134(.a(new_n221), .b(new_n229), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n228), .c(new_n225), .d(new_n221), .o1(\s[20] ));
  nand23aa1n06x5               g136(.a(new_n216), .b(new_n220), .c(new_n228), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nanb03aa1n03x5               g138(.a(new_n226), .b(new_n227), .c(new_n219), .out0(new_n234));
  oai112aa1n03x5               g139(.a(new_n225), .b(new_n212), .c(new_n211), .d(new_n200), .o1(new_n235));
  tech160nm_fiaoi012aa1n04x5   g140(.a(new_n226), .b(new_n218), .c(new_n227), .o1(new_n236));
  oai012aa1n06x5               g141(.a(new_n236), .b(new_n235), .c(new_n234), .o1(new_n237));
  nor042aa1n12x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n237), .c(new_n199), .d(new_n233), .o1(new_n241));
  nano22aa1n03x7               g146(.a(new_n226), .b(new_n219), .c(new_n227), .out0(new_n242));
  oai012aa1n02x5               g147(.a(new_n212), .b(\b[18] ), .c(\a[19] ), .o1(new_n243));
  oab012aa1n04x5               g148(.a(new_n243), .b(new_n200), .c(new_n211), .out0(new_n244));
  inv000aa1n02x5               g149(.a(new_n236), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n245), .b(new_n240), .c(new_n244), .d(new_n242), .o1(new_n246));
  aobi12aa1n02x5               g151(.a(new_n246), .b(new_n199), .c(new_n233), .out0(new_n247));
  norb02aa1n02x5               g152(.a(new_n241), .b(new_n247), .out0(\s[21] ));
  inv000aa1d42x5               g153(.a(new_n238), .o1(new_n249));
  nor002aa1n03x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  nand02aa1n06x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  norb03aa1n02x5               g157(.a(new_n251), .b(new_n238), .c(new_n250), .out0(new_n253));
  nand42aa1n02x5               g158(.a(new_n241), .b(new_n253), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n249), .d(new_n241), .o1(\s[22] ));
  nano23aa1n06x5               g160(.a(new_n238), .b(new_n250), .c(new_n251), .d(new_n239), .out0(new_n256));
  norb02aa1n02x7               g161(.a(new_n256), .b(new_n232), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n256), .b(new_n245), .c(new_n244), .d(new_n242), .o1(new_n258));
  aoi012aa1n12x5               g163(.a(new_n250), .b(new_n238), .c(new_n251), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n258), .b(new_n259), .o1(new_n260));
  nor002aa1d24x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  nand42aa1n03x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n260), .c(new_n199), .d(new_n257), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n259), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n258), .b(new_n265), .c(new_n263), .out0(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n199), .c(new_n257), .o1(new_n267));
  norb02aa1n03x4               g172(.a(new_n264), .b(new_n267), .out0(\s[23] ));
  inv040aa1n04x5               g173(.a(new_n261), .o1(new_n269));
  nor042aa1n03x5               g174(.a(\b[23] ), .b(\a[24] ), .o1(new_n270));
  and002aa1n12x5               g175(.a(\b[23] ), .b(\a[24] ), .o(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n270), .o1(new_n272));
  norp03aa1n02x5               g177(.a(new_n271), .b(new_n270), .c(new_n261), .o1(new_n273));
  nanp02aa1n06x5               g178(.a(new_n264), .b(new_n273), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n272), .c(new_n269), .d(new_n264), .o1(\s[24] ));
  nano32aa1n02x5               g180(.a(new_n232), .b(new_n272), .c(new_n256), .d(new_n263), .out0(new_n276));
  nano23aa1n09x5               g181(.a(new_n271), .b(new_n270), .c(new_n269), .d(new_n262), .out0(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  oab012aa1n04x5               g183(.a(new_n270), .b(new_n269), .c(new_n271), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n278), .c(new_n258), .d(new_n259), .o1(new_n280));
  nor042aa1n03x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  nand42aa1n02x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  norb02aa1n02x5               g187(.a(new_n282), .b(new_n281), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n280), .c(new_n199), .d(new_n276), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n277), .b(new_n265), .c(new_n237), .d(new_n256), .o1(new_n285));
  nanb03aa1n02x5               g190(.a(new_n283), .b(new_n285), .c(new_n279), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n199), .c(new_n276), .o1(new_n287));
  norb02aa1n02x5               g192(.a(new_n284), .b(new_n287), .out0(\s[25] ));
  inv040aa1n02x5               g193(.a(new_n281), .o1(new_n289));
  nor042aa1n03x5               g194(.a(\b[25] ), .b(\a[26] ), .o1(new_n290));
  and002aa1n06x5               g195(.a(\b[25] ), .b(\a[26] ), .o(new_n291));
  norp02aa1n02x5               g196(.a(new_n291), .b(new_n290), .o1(new_n292));
  norp03aa1n02x5               g197(.a(new_n291), .b(new_n290), .c(new_n281), .o1(new_n293));
  nand42aa1n02x5               g198(.a(new_n284), .b(new_n293), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n292), .c(new_n289), .d(new_n284), .o1(\s[26] ));
  nano23aa1n03x7               g200(.a(new_n291), .b(new_n290), .c(new_n289), .d(new_n282), .out0(new_n296));
  inv000aa1n04x5               g201(.a(new_n296), .o1(new_n297));
  nano23aa1n06x5               g202(.a(new_n297), .b(new_n232), .c(new_n277), .d(new_n256), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n209), .c(new_n120), .d(new_n193), .o1(new_n299));
  aobi12aa1n06x5               g204(.a(new_n298), .b(new_n194), .c(new_n198), .out0(new_n300));
  oab012aa1n02x4               g205(.a(new_n290), .b(new_n289), .c(new_n291), .out0(new_n301));
  aoai13aa1n04x5               g206(.a(new_n301), .b(new_n297), .c(new_n285), .d(new_n279), .o1(new_n302));
  xorc02aa1n12x5               g207(.a(\a[27] ), .b(\b[26] ), .out0(new_n303));
  oai012aa1n06x5               g208(.a(new_n303), .b(new_n302), .c(new_n300), .o1(new_n304));
  inv000aa1n02x5               g209(.a(new_n301), .o1(new_n305));
  aoi112aa1n02x5               g210(.a(new_n303), .b(new_n305), .c(new_n280), .d(new_n296), .o1(new_n306));
  aobi12aa1n02x7               g211(.a(new_n304), .b(new_n306), .c(new_n299), .out0(\s[27] ));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n308), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[28] ), .b(\b[27] ), .out0(new_n310));
  tech160nm_fiaoi012aa1n05x5   g215(.a(new_n305), .b(new_n280), .c(new_n296), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n303), .o1(new_n312));
  oai022aa1n02x5               g217(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(\a[28] ), .c(\b[27] ), .o1(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n312), .c(new_n311), .d(new_n299), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n310), .c(new_n304), .d(new_n309), .o1(\s[28] ));
  and002aa1n02x5               g221(.a(new_n310), .b(new_n303), .o(new_n317));
  oaih12aa1n02x5               g222(.a(new_n317), .b(new_n302), .c(new_n300), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  aob012aa1n02x5               g224(.a(new_n313), .b(\b[27] ), .c(\a[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n319), .out0(new_n321));
  inv000aa1d42x5               g226(.a(new_n317), .o1(new_n322));
  aoai13aa1n02x7               g227(.a(new_n320), .b(new_n322), .c(new_n311), .d(new_n299), .o1(new_n323));
  aoi022aa1n03x5               g228(.a(new_n323), .b(new_n319), .c(new_n318), .d(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g230(.a(new_n312), .b(new_n310), .c(new_n319), .out0(new_n326));
  oaih12aa1n02x5               g231(.a(new_n326), .b(new_n302), .c(new_n300), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  inv000aa1d42x5               g233(.a(\a[29] ), .o1(new_n329));
  inv000aa1d42x5               g234(.a(\b[28] ), .o1(new_n330));
  oaib12aa1n02x5               g235(.a(new_n320), .b(\b[28] ), .c(new_n329), .out0(new_n331));
  oaoi13aa1n02x5               g236(.a(new_n328), .b(new_n331), .c(new_n329), .d(new_n330), .o1(new_n332));
  inv000aa1n02x5               g237(.a(new_n326), .o1(new_n333));
  oaib12aa1n02x5               g238(.a(new_n331), .b(new_n330), .c(\a[29] ), .out0(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n333), .c(new_n311), .d(new_n299), .o1(new_n335));
  aoi022aa1n03x5               g240(.a(new_n335), .b(new_n328), .c(new_n327), .d(new_n332), .o1(\s[30] ));
  nano32aa1n02x4               g241(.a(new_n312), .b(new_n328), .c(new_n310), .d(new_n319), .out0(new_n337));
  oaih12aa1n02x5               g242(.a(new_n337), .b(new_n302), .c(new_n300), .o1(new_n338));
  aoi022aa1n02x5               g243(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n339));
  norb02aa1n02x5               g244(.a(\a[31] ), .b(\b[30] ), .out0(new_n340));
  obai22aa1n02x7               g245(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n341));
  aoi112aa1n02x5               g246(.a(new_n341), .b(new_n340), .c(new_n331), .d(new_n339), .o1(new_n342));
  xorc02aa1n02x5               g247(.a(\a[31] ), .b(\b[30] ), .out0(new_n343));
  inv000aa1n02x5               g248(.a(new_n337), .o1(new_n344));
  norp02aa1n02x5               g249(.a(\b[29] ), .b(\a[30] ), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n331), .c(new_n339), .o1(new_n346));
  aoai13aa1n03x5               g251(.a(new_n346), .b(new_n344), .c(new_n311), .d(new_n299), .o1(new_n347));
  aoi022aa1n03x5               g252(.a(new_n347), .b(new_n343), .c(new_n338), .d(new_n342), .o1(\s[31] ));
  aoi022aa1n02x5               g253(.a(new_n131), .b(new_n130), .c(\a[1] ), .d(\b[0] ), .o1(new_n349));
  oaib12aa1n02x5               g254(.a(new_n349), .b(new_n131), .c(\a[2] ), .out0(new_n350));
  norb02aa1n02x5               g255(.a(new_n103), .b(new_n102), .out0(new_n351));
  aboi22aa1n03x5               g256(.a(new_n102), .b(new_n103), .c(new_n130), .d(new_n131), .out0(new_n352));
  aoi022aa1n02x5               g257(.a(new_n350), .b(new_n352), .c(new_n99), .d(new_n351), .o1(\s[3] ));
  obai22aa1n02x7               g258(.a(new_n101), .b(new_n100), .c(\a[3] ), .d(\b[2] ), .out0(new_n354));
  aoi012aa1n02x5               g259(.a(new_n354), .b(new_n99), .c(new_n351), .o1(new_n355));
  oaoi13aa1n02x5               g260(.a(new_n355), .b(new_n134), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorc02aa1n02x5               g261(.a(\a[5] ), .b(\b[4] ), .out0(new_n357));
  xnbna2aa1n03x5               g262(.a(new_n357), .b(new_n105), .c(new_n106), .out0(\s[5] ));
  inv000aa1d42x5               g263(.a(new_n107), .o1(new_n359));
  aoai13aa1n06x5               g264(.a(new_n359), .b(new_n115), .c(new_n134), .d(new_n357), .o1(new_n360));
  aoi112aa1n02x5               g265(.a(new_n115), .b(new_n359), .c(new_n134), .d(new_n357), .o1(new_n361));
  norb02aa1n02x5               g266(.a(new_n360), .b(new_n361), .out0(\s[6] ));
  inv000aa1d42x5               g267(.a(\a[6] ), .o1(new_n363));
  inv000aa1d42x5               g268(.a(\b[5] ), .o1(new_n364));
  nanp02aa1n02x5               g269(.a(new_n364), .b(new_n363), .o1(new_n365));
  nanb02aa1n02x5               g270(.a(new_n111), .b(new_n112), .out0(new_n366));
  xobna2aa1n03x5               g271(.a(new_n366), .b(new_n360), .c(new_n365), .out0(\s[7] ));
  nanb02aa1n02x5               g272(.a(new_n109), .b(new_n110), .out0(new_n368));
  nanb03aa1n02x5               g273(.a(new_n366), .b(new_n360), .c(new_n365), .out0(new_n369));
  xnbna2aa1n03x5               g274(.a(new_n368), .b(new_n369), .c(new_n112), .out0(\s[8] ));
  nano22aa1n02x4               g275(.a(new_n123), .b(new_n137), .c(new_n118), .out0(new_n371));
  aoi022aa1n02x5               g276(.a(new_n120), .b(new_n123), .c(new_n371), .d(new_n154), .o1(\s[9] ));
endmodule


