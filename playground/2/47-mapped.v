// Benchmark "adder" written by ABC on Wed Jul 17 13:22:49 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n225, new_n226, new_n227, new_n228, new_n229, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n332, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n345, new_n346, new_n347, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n355, new_n357, new_n358,
    new_n360, new_n361, new_n363, new_n364, new_n365, new_n367, new_n368,
    new_n370, new_n372, new_n373, new_n375;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanb02aa1d24x5               g003(.a(\a[9] ), .b(new_n98), .out0(new_n99));
  orn002aa1n24x5               g004(.a(\a[2] ), .b(\b[1] ), .o(new_n100));
  nanp02aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  and002aa1n12x5               g006(.a(\b[0] ), .b(\a[1] ), .o(new_n102));
  nanp03aa1n03x5               g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano22aa1n03x5               g010(.a(new_n105), .b(new_n101), .c(new_n104), .out0(new_n106));
  oai022aa1d18x5               g011(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n107));
  aoi012aa1n06x5               g012(.a(new_n107), .b(new_n103), .c(new_n106), .o1(new_n108));
  nand42aa1n08x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nand42aa1n10x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nano22aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n110), .out0(new_n112));
  nand02aa1n03x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  oai012aa1n02x5               g018(.a(new_n113), .b(\b[7] ), .c(\a[8] ), .o1(new_n114));
  norp02aa1n24x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  tech160nm_fiaoi012aa1n03p5x5 g020(.a(new_n115), .b(\a[8] ), .c(\b[7] ), .o1(new_n116));
  nand42aa1n08x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  tech160nm_fioai012aa1n04x5   g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .o1(new_n118));
  nona23aa1n03x5               g023(.a(new_n112), .b(new_n116), .c(new_n118), .d(new_n114), .out0(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  nanb02aa1d24x5               g025(.a(\a[6] ), .b(new_n120), .out0(new_n121));
  oai112aa1n06x5               g026(.a(new_n121), .b(new_n110), .c(\b[4] ), .d(\a[5] ), .o1(new_n122));
  tech160nm_fixorc02aa1n05x5   g027(.a(\a[8] ), .b(\b[7] ), .out0(new_n123));
  nano22aa1n03x7               g028(.a(new_n115), .b(new_n110), .c(new_n117), .out0(new_n124));
  inv000aa1d42x5               g029(.a(\a[8] ), .o1(new_n125));
  inv000aa1d42x5               g030(.a(\b[7] ), .o1(new_n126));
  oao003aa1n02x5               g031(.a(new_n125), .b(new_n126), .c(new_n115), .carry(new_n127));
  aoi013aa1n09x5               g032(.a(new_n127), .b(new_n124), .c(new_n123), .d(new_n122), .o1(new_n128));
  oai012aa1n06x5               g033(.a(new_n128), .b(new_n108), .c(new_n119), .o1(new_n129));
  oaib12aa1n06x5               g034(.a(new_n129), .b(new_n98), .c(\a[9] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n97), .b(new_n130), .c(new_n99), .out0(\s[10] ));
  orn002aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .o(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  oai112aa1n02x5               g038(.a(new_n132), .b(new_n133), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  nanb02aa1n06x5               g039(.a(new_n134), .b(new_n130), .out0(new_n135));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  nand42aa1n03x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  aoi022aa1n02x5               g043(.a(new_n135), .b(new_n133), .c(new_n137), .d(new_n138), .o1(new_n139));
  aoi022aa1d24x5               g044(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n140));
  aoi013aa1n02x4               g045(.a(new_n139), .b(new_n137), .c(new_n135), .d(new_n140), .o1(\s[11] ));
  nand23aa1n03x5               g046(.a(new_n135), .b(new_n137), .c(new_n140), .o1(new_n142));
  nor002aa1d32x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1n10x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  norb03aa1n02x5               g050(.a(new_n144), .b(new_n136), .c(new_n143), .out0(new_n146));
  nand22aa1n03x5               g051(.a(new_n142), .b(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n145), .c(new_n137), .d(new_n142), .o1(\s[12] ));
  nona23aa1n09x5               g053(.a(new_n138), .b(new_n144), .c(new_n143), .d(new_n136), .out0(new_n149));
  xnrc02aa1n12x5               g054(.a(\b[8] ), .b(\a[9] ), .out0(new_n150));
  norb03aa1n12x5               g055(.a(new_n97), .b(new_n149), .c(new_n150), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n129), .b(new_n151), .o1(new_n152));
  nanb03aa1n12x5               g057(.a(new_n105), .b(new_n101), .c(new_n104), .out0(new_n153));
  aoi013aa1n09x5               g058(.a(new_n153), .b(new_n102), .c(new_n101), .d(new_n100), .o1(new_n154));
  nanb03aa1n02x5               g059(.a(new_n111), .b(new_n109), .c(new_n110), .out0(new_n155));
  aoi022aa1n02x5               g060(.a(new_n126), .b(new_n125), .c(\a[4] ), .d(\b[3] ), .o1(new_n156));
  nano23aa1n09x5               g061(.a(new_n118), .b(new_n155), .c(new_n156), .d(new_n116), .out0(new_n157));
  oai012aa1n18x5               g062(.a(new_n157), .b(new_n154), .c(new_n107), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n151), .o1(new_n159));
  oai022aa1n02x5               g064(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n144), .b(new_n160), .c(new_n134), .d(new_n140), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n159), .c(new_n158), .d(new_n128), .o1(new_n162));
  nor002aa1d32x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanp02aa1n12x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  oaih12aa1n02x5               g070(.a(new_n144), .b(new_n143), .c(new_n136), .o1(new_n166));
  oaib12aa1n02x5               g071(.a(new_n166), .b(new_n163), .c(new_n164), .out0(new_n167));
  aoi013aa1n02x4               g072(.a(new_n167), .b(new_n134), .c(new_n146), .d(new_n140), .o1(new_n168));
  aoi022aa1n02x5               g073(.a(new_n162), .b(new_n165), .c(new_n152), .d(new_n168), .o1(\s[13] ));
  inv000aa1d42x5               g074(.a(new_n163), .o1(new_n170));
  nona23aa1n03x5               g075(.a(new_n140), .b(new_n144), .c(new_n143), .d(new_n136), .out0(new_n171));
  aoai13aa1n03x5               g076(.a(new_n166), .b(new_n171), .c(new_n97), .d(new_n99), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n165), .b(new_n172), .c(new_n129), .d(new_n151), .o1(new_n173));
  nor002aa1d32x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1d28x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  norb03aa1d15x5               g081(.a(new_n175), .b(new_n163), .c(new_n174), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n173), .b(new_n177), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n170), .d(new_n173), .o1(\s[14] ));
  nano23aa1n03x7               g084(.a(new_n163), .b(new_n174), .c(new_n175), .d(new_n164), .out0(new_n180));
  oaoi03aa1n02x5               g085(.a(\a[14] ), .b(\b[13] ), .c(new_n170), .o1(new_n181));
  nor042aa1d18x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand02aa1d04x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n03x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n181), .c(new_n162), .d(new_n180), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n181), .c(new_n162), .d(new_n180), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(\s[15] ));
  inv000aa1d42x5               g092(.a(new_n182), .o1(new_n188));
  nor042aa1n06x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand42aa1d28x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  nona23aa1n02x4               g096(.a(new_n185), .b(new_n190), .c(new_n189), .d(new_n182), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n188), .d(new_n185), .o1(\s[16] ));
  nano23aa1n02x4               g098(.a(new_n143), .b(new_n136), .c(new_n144), .d(new_n138), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n150), .o1(new_n195));
  nanp03aa1n02x5               g100(.a(new_n180), .b(new_n184), .c(new_n191), .o1(new_n196));
  nano32aa1n06x5               g101(.a(new_n196), .b(new_n195), .c(new_n194), .d(new_n97), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n129), .b(new_n197), .o1(new_n198));
  nona23aa1n02x4               g103(.a(new_n175), .b(new_n164), .c(new_n163), .d(new_n174), .out0(new_n199));
  nano22aa1n03x7               g104(.a(new_n199), .b(new_n184), .c(new_n191), .out0(new_n200));
  nand22aa1n03x5               g105(.a(new_n151), .b(new_n200), .o1(new_n201));
  oai012aa1n02x5               g106(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .o1(new_n202));
  nanb03aa1n02x5               g107(.a(new_n189), .b(new_n190), .c(new_n183), .out0(new_n203));
  tech160nm_fioai012aa1n03p5x5 g108(.a(new_n190), .b(new_n189), .c(new_n182), .o1(new_n204));
  oai013aa1n02x4               g109(.a(new_n204), .b(new_n177), .c(new_n203), .d(new_n202), .o1(new_n205));
  tech160nm_fiaoi012aa1n05x5   g110(.a(new_n205), .b(new_n172), .c(new_n200), .o1(new_n206));
  aoai13aa1n12x5               g111(.a(new_n206), .b(new_n201), .c(new_n158), .d(new_n128), .o1(new_n207));
  xorc02aa1n12x5               g112(.a(\a[17] ), .b(\b[16] ), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n177), .o1(new_n209));
  nano23aa1n02x4               g114(.a(new_n202), .b(new_n189), .c(new_n183), .d(new_n190), .out0(new_n210));
  nanb02aa1n02x5               g115(.a(new_n208), .b(new_n204), .out0(new_n211));
  aoi122aa1n02x5               g116(.a(new_n211), .b(new_n209), .c(new_n210), .d(new_n172), .e(new_n200), .o1(new_n212));
  aoi022aa1n02x5               g117(.a(new_n207), .b(new_n208), .c(new_n198), .d(new_n212), .o1(\s[17] ));
  nor002aa1d32x5               g118(.a(\b[16] ), .b(\a[17] ), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  aobi12aa1n02x5               g120(.a(new_n204), .b(new_n210), .c(new_n209), .out0(new_n216));
  tech160nm_fioai012aa1n05x5   g121(.a(new_n216), .b(new_n161), .c(new_n196), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n208), .b(new_n217), .c(new_n129), .d(new_n197), .o1(new_n218));
  norp02aa1n12x5               g123(.a(\b[17] ), .b(\a[18] ), .o1(new_n219));
  nand42aa1n16x5               g124(.a(\b[17] ), .b(\a[18] ), .o1(new_n220));
  norb02aa1n06x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  norp02aa1n02x5               g126(.a(new_n219), .b(new_n214), .o1(new_n222));
  nanp03aa1n02x5               g127(.a(new_n218), .b(new_n220), .c(new_n222), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n221), .c(new_n218), .d(new_n215), .o1(\s[18] ));
  and002aa1n02x5               g129(.a(new_n208), .b(new_n221), .o(new_n225));
  oaoi03aa1n02x5               g130(.a(\a[18] ), .b(\b[17] ), .c(new_n215), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[19] ), .b(\b[18] ), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n226), .c(new_n207), .d(new_n225), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(new_n227), .b(new_n226), .c(new_n207), .d(new_n225), .o1(new_n229));
  norb02aa1n03x4               g134(.a(new_n228), .b(new_n229), .out0(\s[19] ));
  xnrc02aa1n02x5               g135(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g136(.a(\a[19] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\b[18] ), .o1(new_n233));
  nanp02aa1n04x5               g138(.a(new_n233), .b(new_n232), .o1(new_n234));
  nor042aa1n06x5               g139(.a(\b[19] ), .b(\a[20] ), .o1(new_n235));
  nanp02aa1n06x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  nano22aa1n02x4               g142(.a(new_n235), .b(new_n234), .c(new_n236), .out0(new_n238));
  tech160nm_finand02aa1n03p5x5 g143(.a(new_n228), .b(new_n238), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n237), .c(new_n234), .d(new_n228), .o1(\s[20] ));
  nand42aa1n02x5               g145(.a(\b[18] ), .b(\a[19] ), .o1(new_n241));
  nano32aa1n03x7               g146(.a(new_n235), .b(new_n234), .c(new_n236), .d(new_n241), .out0(new_n242));
  nand23aa1n06x5               g147(.a(new_n242), .b(new_n208), .c(new_n221), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nanb03aa1n03x5               g149(.a(new_n235), .b(new_n236), .c(new_n241), .out0(new_n245));
  oai112aa1n02x5               g150(.a(new_n220), .b(new_n234), .c(new_n219), .d(new_n214), .o1(new_n246));
  aoai13aa1n12x5               g151(.a(new_n236), .b(new_n235), .c(new_n232), .d(new_n233), .o1(new_n247));
  oai012aa1n04x7               g152(.a(new_n247), .b(new_n246), .c(new_n245), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[21] ), .b(\b[20] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n207), .d(new_n244), .o1(new_n250));
  nano22aa1n02x4               g155(.a(new_n235), .b(new_n241), .c(new_n236), .out0(new_n251));
  oai012aa1n02x5               g156(.a(new_n220), .b(\b[18] ), .c(\a[19] ), .o1(new_n252));
  nona22aa1n03x5               g157(.a(new_n251), .b(new_n222), .c(new_n252), .out0(new_n253));
  nano22aa1n02x4               g158(.a(new_n249), .b(new_n253), .c(new_n247), .out0(new_n254));
  aobi12aa1n02x5               g159(.a(new_n254), .b(new_n207), .c(new_n244), .out0(new_n255));
  norb02aa1n03x4               g160(.a(new_n250), .b(new_n255), .out0(\s[21] ));
  nor042aa1n04x5               g161(.a(\b[20] ), .b(\a[21] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nor042aa1n03x5               g163(.a(\b[21] ), .b(\a[22] ), .o1(new_n259));
  and002aa1n12x5               g164(.a(\b[21] ), .b(\a[22] ), .o(new_n260));
  nor042aa1n04x5               g165(.a(new_n260), .b(new_n259), .o1(new_n261));
  norp03aa1n02x5               g166(.a(new_n260), .b(new_n259), .c(new_n257), .o1(new_n262));
  nand42aa1n02x5               g167(.a(new_n250), .b(new_n262), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n261), .c(new_n258), .d(new_n250), .o1(\s[22] ));
  tech160nm_finand02aa1n05x5   g169(.a(new_n249), .b(new_n261), .o1(new_n265));
  nano32aa1n02x4               g170(.a(new_n265), .b(new_n242), .c(new_n221), .d(new_n208), .out0(new_n266));
  oab012aa1d15x5               g171(.a(new_n260), .b(new_n257), .c(new_n259), .out0(new_n267));
  inv040aa1n03x5               g172(.a(new_n267), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n265), .c(new_n253), .d(new_n247), .o1(new_n269));
  nor022aa1n16x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  nand42aa1n02x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  norb02aa1n02x5               g176(.a(new_n271), .b(new_n270), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n269), .c(new_n207), .d(new_n266), .o1(new_n273));
  inv040aa1n03x5               g178(.a(new_n265), .o1(new_n274));
  aoi112aa1n02x5               g179(.a(new_n272), .b(new_n267), .c(new_n248), .d(new_n274), .o1(new_n275));
  aobi12aa1n02x5               g180(.a(new_n275), .b(new_n207), .c(new_n266), .out0(new_n276));
  norb02aa1n03x4               g181(.a(new_n273), .b(new_n276), .out0(\s[23] ));
  inv030aa1n02x5               g182(.a(new_n270), .o1(new_n278));
  nor042aa1n02x5               g183(.a(\b[23] ), .b(\a[24] ), .o1(new_n279));
  and002aa1n02x5               g184(.a(\b[23] ), .b(\a[24] ), .o(new_n280));
  norp02aa1n02x5               g185(.a(new_n280), .b(new_n279), .o1(new_n281));
  norp03aa1n02x5               g186(.a(new_n280), .b(new_n279), .c(new_n270), .o1(new_n282));
  nand42aa1n02x5               g187(.a(new_n273), .b(new_n282), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n281), .c(new_n278), .d(new_n273), .o1(\s[24] ));
  nano23aa1n06x5               g189(.a(new_n280), .b(new_n279), .c(new_n278), .d(new_n271), .out0(new_n285));
  nano32aa1n02x5               g190(.a(new_n243), .b(new_n285), .c(new_n249), .d(new_n261), .out0(new_n286));
  aobi12aa1n02x5               g191(.a(new_n286), .b(new_n198), .c(new_n206), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n285), .b(new_n267), .c(new_n248), .d(new_n274), .o1(new_n288));
  oab012aa1n06x5               g193(.a(new_n280), .b(new_n270), .c(new_n279), .out0(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  nanp02aa1n02x5               g195(.a(new_n288), .b(new_n290), .o1(new_n291));
  tech160nm_fixorc02aa1n03p5x5 g196(.a(\a[25] ), .b(\b[24] ), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n291), .c(new_n207), .d(new_n286), .o1(new_n293));
  nona22aa1n02x4               g198(.a(new_n288), .b(new_n289), .c(new_n292), .out0(new_n294));
  oa0012aa1n03x5               g199(.a(new_n293), .b(new_n294), .c(new_n287), .o(\s[25] ));
  norp02aa1n09x5               g200(.a(\b[24] ), .b(\a[25] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  norp02aa1n02x5               g202(.a(\b[25] ), .b(\a[26] ), .o1(new_n298));
  and002aa1n03x5               g203(.a(\b[25] ), .b(\a[26] ), .o(new_n299));
  nor002aa1n02x5               g204(.a(new_n299), .b(new_n298), .o1(new_n300));
  norp03aa1n02x5               g205(.a(new_n299), .b(new_n298), .c(new_n296), .o1(new_n301));
  nand42aa1n02x5               g206(.a(new_n293), .b(new_n301), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n300), .c(new_n297), .d(new_n293), .o1(\s[26] ));
  and002aa1n02x5               g208(.a(new_n292), .b(new_n300), .o(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n289), .c(new_n269), .d(new_n285), .o1(new_n305));
  nano32aa1n03x7               g210(.a(new_n243), .b(new_n304), .c(new_n274), .d(new_n285), .out0(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n217), .c(new_n129), .d(new_n197), .o1(new_n307));
  oab012aa1n02x4               g212(.a(new_n299), .b(new_n296), .c(new_n298), .out0(new_n308));
  inv000aa1n02x5               g213(.a(new_n308), .o1(new_n309));
  nand23aa1n04x5               g214(.a(new_n307), .b(new_n305), .c(new_n309), .o1(new_n310));
  xorc02aa1n12x5               g215(.a(\a[27] ), .b(\b[26] ), .out0(new_n311));
  aoi112aa1n03x4               g216(.a(new_n311), .b(new_n308), .c(new_n207), .d(new_n306), .o1(new_n312));
  aoi022aa1n02x5               g217(.a(new_n312), .b(new_n305), .c(new_n310), .d(new_n311), .o1(\s[27] ));
  norp02aa1n02x5               g218(.a(\b[26] ), .b(\a[27] ), .o1(new_n314));
  xnrc02aa1n12x5               g219(.a(\b[27] ), .b(\a[28] ), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n315), .b(new_n314), .c(new_n310), .d(new_n311), .o1(new_n316));
  aobi12aa1n02x5               g221(.a(new_n304), .b(new_n288), .c(new_n290), .out0(new_n317));
  aoi112aa1n06x5               g222(.a(new_n317), .b(new_n308), .c(new_n207), .d(new_n306), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n311), .o1(new_n319));
  oai022aa1d18x5               g224(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n320));
  aoi012aa1n02x5               g225(.a(new_n320), .b(\a[28] ), .c(\b[27] ), .o1(new_n321));
  oai012aa1n03x5               g226(.a(new_n321), .b(new_n318), .c(new_n319), .o1(new_n322));
  nanp02aa1n03x5               g227(.a(new_n316), .b(new_n322), .o1(\s[28] ));
  xnrc02aa1n02x5               g228(.a(\b[28] ), .b(\a[29] ), .out0(new_n324));
  norb02aa1d27x5               g229(.a(new_n311), .b(new_n315), .out0(new_n325));
  inv000aa1d42x5               g230(.a(\a[28] ), .o1(new_n326));
  inv000aa1d42x5               g231(.a(\b[27] ), .o1(new_n327));
  oao003aa1n02x5               g232(.a(new_n326), .b(new_n327), .c(new_n314), .carry(new_n328));
  aoai13aa1n02x5               g233(.a(new_n324), .b(new_n328), .c(new_n310), .d(new_n325), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n325), .o1(new_n330));
  norp02aa1n02x5               g235(.a(new_n328), .b(new_n324), .o1(new_n331));
  tech160nm_fioai012aa1n05x5   g236(.a(new_n331), .b(new_n318), .c(new_n330), .o1(new_n332));
  nanp02aa1n03x5               g237(.a(new_n329), .b(new_n332), .o1(\s[29] ));
  xnrb03aa1n02x5               g238(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g239(.a(new_n311), .b(new_n324), .c(new_n315), .out0(new_n335));
  nand42aa1n02x5               g240(.a(\b[28] ), .b(\a[29] ), .o1(new_n336));
  oai112aa1n06x5               g241(.a(new_n320), .b(new_n336), .c(new_n327), .d(new_n326), .o1(new_n337));
  oai012aa1n02x5               g242(.a(new_n337), .b(\b[28] ), .c(\a[29] ), .o1(new_n338));
  norp02aa1n02x5               g243(.a(\b[29] ), .b(\a[30] ), .o1(new_n339));
  nanp02aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .o1(new_n340));
  nanb02aa1n02x5               g245(.a(new_n339), .b(new_n340), .out0(new_n341));
  aoai13aa1n02x5               g246(.a(new_n341), .b(new_n338), .c(new_n310), .d(new_n335), .o1(new_n342));
  inv000aa1d42x5               g247(.a(new_n335), .o1(new_n343));
  norp02aa1n02x5               g248(.a(\b[28] ), .b(\a[29] ), .o1(new_n344));
  nona23aa1n06x5               g249(.a(new_n337), .b(new_n340), .c(new_n339), .d(new_n344), .out0(new_n345));
  inv000aa1d42x5               g250(.a(new_n345), .o1(new_n346));
  tech160nm_fioai012aa1n05x5   g251(.a(new_n346), .b(new_n318), .c(new_n343), .o1(new_n347));
  nanp02aa1n03x5               g252(.a(new_n342), .b(new_n347), .o1(\s[30] ));
  norb03aa1n03x5               g253(.a(new_n325), .b(new_n324), .c(new_n341), .out0(new_n349));
  and002aa1n02x5               g254(.a(new_n345), .b(new_n340), .o(new_n350));
  xnrc02aa1n02x5               g255(.a(\b[30] ), .b(\a[31] ), .out0(new_n351));
  aoai13aa1n02x5               g256(.a(new_n351), .b(new_n350), .c(new_n310), .d(new_n349), .o1(new_n352));
  inv000aa1d42x5               g257(.a(new_n349), .o1(new_n353));
  aoi012aa1n02x5               g258(.a(new_n351), .b(new_n345), .c(new_n340), .o1(new_n354));
  oai012aa1n03x5               g259(.a(new_n354), .b(new_n318), .c(new_n353), .o1(new_n355));
  nanp02aa1n03x5               g260(.a(new_n352), .b(new_n355), .o1(\s[31] ));
  inv000aa1d42x5               g261(.a(new_n105), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n103), .b(new_n101), .c(new_n357), .d(new_n104), .o1(new_n358));
  norp02aa1n02x5               g263(.a(new_n358), .b(new_n154), .o1(\s[3] ));
  inv000aa1d42x5               g264(.a(new_n154), .o1(new_n360));
  xorc02aa1n02x5               g265(.a(\a[4] ), .b(\b[3] ), .out0(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n361), .b(new_n360), .c(new_n357), .out0(\s[4] ));
  nona23aa1n03x5               g267(.a(new_n113), .b(new_n109), .c(new_n108), .d(new_n111), .out0(new_n363));
  inv000aa1d42x5               g268(.a(new_n111), .o1(new_n364));
  aboi22aa1n03x5               g269(.a(new_n108), .b(new_n113), .c(new_n109), .d(new_n364), .out0(new_n365));
  norb02aa1n02x5               g270(.a(new_n363), .b(new_n365), .out0(\s[5] ));
  norb02aa1n02x5               g271(.a(new_n363), .b(new_n111), .out0(new_n367));
  nanb02aa1n03x5               g272(.a(new_n122), .b(new_n363), .out0(new_n368));
  aoai13aa1n02x5               g273(.a(new_n368), .b(new_n367), .c(new_n110), .d(new_n121), .o1(\s[6] ));
  nanb02aa1n02x5               g274(.a(new_n115), .b(new_n117), .out0(new_n370));
  xnbna2aa1n03x5               g275(.a(new_n370), .b(new_n368), .c(new_n110), .out0(\s[7] ));
  orn002aa1n02x5               g276(.a(\a[7] ), .b(\b[6] ), .o(new_n372));
  nanp02aa1n03x5               g277(.a(new_n368), .b(new_n124), .o1(new_n373));
  xnbna2aa1n03x5               g278(.a(new_n123), .b(new_n373), .c(new_n372), .out0(\s[8] ));
  aoi113aa1n02x5               g279(.a(new_n195), .b(new_n127), .c(new_n124), .d(new_n123), .e(new_n122), .o1(new_n375));
  aoi022aa1n02x5               g280(.a(new_n129), .b(new_n195), .c(new_n158), .d(new_n375), .o1(\s[9] ));
endmodule


