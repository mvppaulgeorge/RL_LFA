// Benchmark "adder" written by ABC on Thu Jul 18 06:28:02 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n337,
    new_n338, new_n339, new_n341, new_n342, new_n343, new_n344, new_n346,
    new_n347, new_n349, new_n350, new_n351, new_n353, new_n354, new_n355,
    new_n357, new_n359, new_n360, new_n361;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\b[1] ), .o1(new_n100));
  nand42aa1d28x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n12x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1n20x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n12x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n04x5   g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1d24x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nand42aa1n08x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor022aa1n12x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  tech160nm_finand02aa1n03p5x5 g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n110), .b(new_n113), .c(new_n112), .d(new_n111), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  nor022aa1n16x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand42aa1n10x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanb02aa1n12x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  nor043aa1n09x5               g023(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n119));
  inv000aa1n02x5               g024(.a(new_n116), .o1(new_n120));
  oao003aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .carry(new_n121));
  nanb03aa1n06x5               g026(.a(new_n116), .b(new_n117), .c(new_n110), .out0(new_n122));
  nor002aa1n02x5               g027(.a(new_n112), .b(new_n111), .o1(new_n123));
  oai013aa1n06x5               g028(.a(new_n121), .b(new_n122), .c(new_n115), .d(new_n123), .o1(new_n124));
  nand42aa1d28x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n09x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n127));
  nor042aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1d28x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  norp02aa1n02x5               g035(.a(new_n128), .b(new_n97), .o1(new_n131));
  nanp03aa1n02x5               g036(.a(new_n127), .b(new_n129), .c(new_n131), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n130), .c(new_n98), .d(new_n127), .o1(\s[10] ));
  nano23aa1n02x5               g038(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n134));
  aoai13aa1n03x5               g039(.a(new_n134), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n135));
  oai012aa1n02x5               g040(.a(new_n129), .b(new_n128), .c(new_n97), .o1(new_n136));
  nor002aa1d32x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n135), .c(new_n136), .out0(\s[11] ));
  inv040aa1n02x5               g045(.a(new_n137), .o1(new_n141));
  aob012aa1n03x5               g046(.a(new_n139), .b(new_n135), .c(new_n136), .out0(new_n142));
  nor002aa1n08x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand42aa1d28x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nona23aa1n03x5               g050(.a(new_n142), .b(new_n144), .c(new_n143), .d(new_n137), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n142), .d(new_n141), .o1(\s[12] ));
  inv000aa1d42x5               g052(.a(new_n126), .o1(new_n148));
  nano32aa1n03x7               g053(.a(new_n148), .b(new_n145), .c(new_n130), .d(new_n139), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n150));
  oai012aa1n12x5               g055(.a(new_n144), .b(new_n143), .c(new_n137), .o1(new_n151));
  nano22aa1n03x7               g056(.a(new_n143), .b(new_n138), .c(new_n144), .out0(new_n152));
  oai112aa1n06x5               g057(.a(new_n141), .b(new_n129), .c(new_n128), .d(new_n97), .o1(new_n153));
  oaib12aa1n18x5               g058(.a(new_n151), .b(new_n153), .c(new_n152), .out0(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  nand02aa1d04x5               g060(.a(new_n150), .b(new_n155), .o1(new_n156));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1n20x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  oaib12aa1n02x5               g064(.a(new_n151), .b(new_n157), .c(new_n158), .out0(new_n160));
  aoib12aa1n02x5               g065(.a(new_n160), .b(new_n152), .c(new_n153), .out0(new_n161));
  aoi022aa1n02x5               g066(.a(new_n156), .b(new_n159), .c(new_n150), .d(new_n161), .o1(\s[13] ));
  inv000aa1d42x5               g067(.a(new_n157), .o1(new_n163));
  nanp02aa1n03x5               g068(.a(new_n156), .b(new_n159), .o1(new_n164));
  nor042aa1n09x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand42aa1n20x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  nona23aa1n03x5               g072(.a(new_n164), .b(new_n166), .c(new_n165), .d(new_n157), .out0(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n167), .c(new_n163), .d(new_n164), .o1(\s[14] ));
  nano23aa1n09x5               g074(.a(new_n157), .b(new_n165), .c(new_n166), .d(new_n158), .out0(new_n170));
  oaoi03aa1n02x5               g075(.a(\a[14] ), .b(\b[13] ), .c(new_n163), .o1(new_n171));
  nor022aa1n16x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n08x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n03x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n171), .c(new_n156), .d(new_n170), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n174), .b(new_n171), .c(new_n156), .d(new_n170), .o1(new_n176));
  norb02aa1n03x4               g081(.a(new_n175), .b(new_n176), .out0(\s[15] ));
  inv020aa1n02x5               g082(.a(new_n172), .o1(new_n178));
  nor042aa1n06x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand42aa1n16x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  norb03aa1n02x5               g086(.a(new_n180), .b(new_n172), .c(new_n179), .out0(new_n182));
  nanp02aa1n03x5               g087(.a(new_n175), .b(new_n182), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n181), .c(new_n175), .d(new_n178), .o1(\s[16] ));
  nano23aa1n02x4               g089(.a(new_n137), .b(new_n143), .c(new_n144), .d(new_n138), .out0(new_n185));
  nano23aa1n03x5               g090(.a(new_n172), .b(new_n179), .c(new_n180), .d(new_n173), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n170), .o1(new_n187));
  nano22aa1n06x5               g092(.a(new_n187), .b(new_n134), .c(new_n185), .out0(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n189));
  nona23aa1n02x4               g094(.a(new_n166), .b(new_n158), .c(new_n157), .d(new_n165), .out0(new_n190));
  nano22aa1n03x7               g095(.a(new_n190), .b(new_n174), .c(new_n181), .out0(new_n191));
  oai012aa1n02x5               g096(.a(new_n180), .b(new_n179), .c(new_n172), .o1(new_n192));
  nanb03aa1n02x5               g097(.a(new_n179), .b(new_n180), .c(new_n173), .out0(new_n193));
  oai112aa1n03x5               g098(.a(new_n178), .b(new_n166), .c(new_n165), .d(new_n157), .o1(new_n194));
  oaih12aa1n02x5               g099(.a(new_n192), .b(new_n194), .c(new_n193), .o1(new_n195));
  aoi012aa1d18x5               g100(.a(new_n195), .b(new_n154), .c(new_n191), .o1(new_n196));
  nand22aa1n09x5               g101(.a(new_n189), .b(new_n196), .o1(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  xnrc02aa1n02x5               g103(.a(\b[16] ), .b(\a[17] ), .out0(new_n199));
  oai112aa1n02x5               g104(.a(new_n192), .b(new_n199), .c(new_n194), .d(new_n193), .o1(new_n200));
  aoi012aa1n02x5               g105(.a(new_n200), .b(new_n154), .c(new_n191), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n197), .b(new_n198), .c(new_n201), .d(new_n189), .o1(\s[17] ));
  nor042aa1n03x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  nanp02aa1n06x5               g109(.a(new_n197), .b(new_n198), .o1(new_n205));
  tech160nm_fixorc02aa1n03p5x5 g110(.a(\a[18] ), .b(\b[17] ), .out0(new_n206));
  nanp02aa1n12x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  oai022aa1d24x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  nanb03aa1n03x5               g113(.a(new_n208), .b(new_n205), .c(new_n207), .out0(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n206), .c(new_n204), .d(new_n205), .o1(\s[18] ));
  norb02aa1n02x5               g115(.a(new_n206), .b(new_n199), .out0(new_n211));
  oaoi03aa1n02x5               g116(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n212));
  xorc02aa1n12x5               g117(.a(\a[19] ), .b(\b[18] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n197), .d(new_n211), .o1(new_n214));
  aoi112aa1n02x7               g119(.a(new_n213), .b(new_n212), .c(new_n197), .d(new_n211), .o1(new_n215));
  norb02aa1n03x4               g120(.a(new_n214), .b(new_n215), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  inv020aa1n02x5               g123(.a(new_n218), .o1(new_n219));
  tech160nm_fixorc02aa1n03p5x5 g124(.a(\a[20] ), .b(\b[19] ), .out0(new_n220));
  nand02aa1n20x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  oai022aa1n03x5               g127(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n223));
  nona22aa1n03x5               g128(.a(new_n214), .b(new_n222), .c(new_n223), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n220), .c(new_n219), .d(new_n214), .o1(\s[20] ));
  nano32aa1n06x5               g130(.a(new_n199), .b(new_n220), .c(new_n206), .d(new_n213), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\a[19] ), .o1(new_n227));
  inv030aa1d32x5               g132(.a(\b[18] ), .o1(new_n228));
  inv020aa1n10x5               g133(.a(\a[20] ), .o1(new_n229));
  nanb02aa1n12x5               g134(.a(\b[19] ), .b(new_n229), .out0(new_n230));
  oai112aa1n06x5               g135(.a(new_n230), .b(new_n221), .c(new_n228), .d(new_n227), .o1(new_n231));
  oai112aa1n06x5               g136(.a(new_n208), .b(new_n207), .c(\b[18] ), .d(\a[19] ), .o1(new_n232));
  nand42aa1n02x5               g137(.a(new_n223), .b(new_n221), .o1(new_n233));
  oai012aa1n09x5               g138(.a(new_n233), .b(new_n232), .c(new_n231), .o1(new_n234));
  tech160nm_fixorc02aa1n03p5x5 g139(.a(\a[21] ), .b(\b[20] ), .out0(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n234), .c(new_n197), .d(new_n226), .o1(new_n236));
  nano32aa1n02x4               g141(.a(new_n231), .b(new_n208), .c(new_n219), .d(new_n207), .out0(new_n237));
  nona22aa1n02x4               g142(.a(new_n233), .b(new_n237), .c(new_n235), .out0(new_n238));
  aoi012aa1n02x7               g143(.a(new_n238), .b(new_n197), .c(new_n226), .o1(new_n239));
  norb02aa1n03x4               g144(.a(new_n236), .b(new_n239), .out0(\s[21] ));
  inv000aa1d42x5               g145(.a(\a[21] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(\b[20] ), .b(new_n241), .out0(new_n242));
  xorc02aa1n02x5               g147(.a(\a[22] ), .b(\b[21] ), .out0(new_n243));
  and002aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .o(new_n244));
  oai022aa1n02x5               g149(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n245));
  nona22aa1n03x5               g150(.a(new_n236), .b(new_n244), .c(new_n245), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n243), .c(new_n242), .d(new_n236), .o1(\s[22] ));
  inv000aa1n02x5               g152(.a(new_n226), .o1(new_n248));
  inv040aa1d32x5               g153(.a(\a[22] ), .o1(new_n249));
  xroi22aa1d06x4               g154(.a(new_n241), .b(\b[20] ), .c(new_n249), .d(\b[21] ), .out0(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n248), .out0(new_n251));
  oaoi03aa1n02x5               g156(.a(\a[22] ), .b(\b[21] ), .c(new_n242), .o1(new_n252));
  tech160nm_fiao0012aa1n02p5x5 g157(.a(new_n252), .b(new_n234), .c(new_n250), .o(new_n253));
  xorc02aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n197), .d(new_n251), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n252), .c(new_n234), .d(new_n250), .o1(new_n256));
  aobi12aa1n02x5               g161(.a(new_n256), .b(new_n197), .c(new_n251), .out0(new_n257));
  norb02aa1n03x4               g162(.a(new_n255), .b(new_n257), .out0(\s[23] ));
  norp02aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .out0(new_n261));
  and002aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o(new_n262));
  oai022aa1n02x5               g167(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n263));
  nona22aa1n03x5               g168(.a(new_n255), .b(new_n262), .c(new_n263), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n261), .c(new_n260), .d(new_n255), .o1(\s[24] ));
  inv000aa1d42x5               g170(.a(\a[23] ), .o1(new_n266));
  inv040aa1d32x5               g171(.a(\a[24] ), .o1(new_n267));
  xroi22aa1d06x4               g172(.a(new_n266), .b(\b[22] ), .c(new_n267), .d(\b[23] ), .out0(new_n268));
  nano22aa1n02x5               g173(.a(new_n248), .b(new_n250), .c(new_n268), .out0(new_n269));
  oaib12aa1n02x5               g174(.a(new_n263), .b(new_n267), .c(\b[23] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n268), .b(new_n252), .c(new_n234), .d(new_n250), .o1(new_n271));
  nanp02aa1n02x5               g176(.a(new_n271), .b(new_n270), .o1(new_n272));
  xorc02aa1n12x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n197), .d(new_n269), .o1(new_n274));
  nanb02aa1n02x5               g179(.a(new_n273), .b(new_n270), .out0(new_n275));
  aoi122aa1n02x7               g180(.a(new_n275), .b(new_n253), .c(new_n268), .d(new_n197), .e(new_n269), .o1(new_n276));
  norb02aa1n03x4               g181(.a(new_n274), .b(new_n276), .out0(\s[25] ));
  norp02aa1n02x5               g182(.a(\b[24] ), .b(\a[25] ), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  tech160nm_fixorc02aa1n05x5   g184(.a(\a[26] ), .b(\b[25] ), .out0(new_n280));
  and002aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .o(new_n281));
  oai022aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n282));
  nona22aa1n03x5               g187(.a(new_n274), .b(new_n281), .c(new_n282), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n280), .c(new_n279), .d(new_n274), .o1(\s[26] ));
  nanp02aa1n02x5               g189(.a(new_n243), .b(new_n235), .o1(new_n285));
  nand02aa1n03x5               g190(.a(new_n280), .b(new_n273), .o1(new_n286));
  nona23aa1n09x5               g191(.a(new_n226), .b(new_n268), .c(new_n286), .d(new_n285), .out0(new_n287));
  nanb02aa1n02x5               g192(.a(new_n287), .b(new_n197), .out0(new_n288));
  aoi012aa1n09x5               g193(.a(new_n287), .b(new_n189), .c(new_n196), .o1(new_n289));
  aob012aa1n02x5               g194(.a(new_n282), .b(\b[25] ), .c(\a[26] ), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n286), .c(new_n271), .d(new_n270), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  oai012aa1n06x5               g197(.a(new_n292), .b(new_n291), .c(new_n289), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n281), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n286), .o1(new_n295));
  aoi122aa1n02x5               g200(.a(new_n292), .b(new_n294), .c(new_n282), .d(new_n272), .e(new_n295), .o1(new_n296));
  aobi12aa1n03x7               g201(.a(new_n293), .b(new_n296), .c(new_n288), .out0(\s[27] ));
  norp02aa1n02x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .out0(new_n300));
  oai022aa1n02x5               g205(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(\a[28] ), .c(\b[27] ), .o1(new_n302));
  nand22aa1n03x5               g207(.a(new_n293), .b(new_n302), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n293), .d(new_n299), .o1(\s[28] ));
  xorc02aa1n12x5               g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  and002aa1n02x5               g210(.a(new_n300), .b(new_n292), .o(new_n306));
  aob012aa1n09x5               g211(.a(new_n301), .b(\b[27] ), .c(\a[28] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  oaoi13aa1n02x7               g213(.a(new_n308), .b(new_n306), .c(new_n291), .d(new_n289), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n305), .o1(new_n310));
  tech160nm_fioai012aa1n02p5x5 g215(.a(new_n306), .b(new_n291), .c(new_n289), .o1(new_n311));
  nona22aa1n03x5               g216(.a(new_n311), .b(new_n308), .c(new_n310), .out0(new_n312));
  oai012aa1n03x5               g217(.a(new_n312), .b(new_n309), .c(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g219(.a(new_n310), .b(new_n292), .c(new_n300), .out0(new_n315));
  oaoi03aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .o1(new_n316));
  oaoi13aa1n02x7               g221(.a(new_n316), .b(new_n315), .c(new_n291), .d(new_n289), .o1(new_n317));
  norp02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .o1(new_n318));
  nanp02aa1n02x5               g223(.a(\b[29] ), .b(\a[30] ), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n319), .b(new_n318), .out0(new_n320));
  tech160nm_fioai012aa1n02p5x5 g225(.a(new_n315), .b(new_n291), .c(new_n289), .o1(new_n321));
  and002aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .o(new_n322));
  nona32aa1n03x5               g227(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n316), .out0(new_n323));
  oai012aa1n03x5               g228(.a(new_n323), .b(new_n317), .c(new_n320), .o1(\s[30] ));
  nano32aa1n02x4               g229(.a(new_n310), .b(new_n300), .c(new_n292), .d(new_n320), .out0(new_n325));
  tech160nm_fioai012aa1n03p5x5 g230(.a(new_n325), .b(new_n291), .c(new_n289), .o1(new_n326));
  oai012aa1n02x5               g231(.a(new_n307), .b(\b[28] ), .c(\a[29] ), .o1(new_n327));
  nanp02aa1n02x5               g232(.a(\b[28] ), .b(\a[29] ), .o1(new_n328));
  nano22aa1n02x4               g233(.a(new_n318), .b(new_n328), .c(new_n319), .out0(new_n329));
  aoi012aa1n02x5               g234(.a(new_n318), .b(new_n327), .c(new_n329), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  oai022aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(\b[30] ), .d(\a[31] ), .o1(new_n332));
  aoi122aa1n02x5               g237(.a(new_n332), .b(\b[30] ), .c(\a[31] ), .d(new_n327), .e(new_n329), .o1(new_n333));
  nanp02aa1n03x5               g238(.a(new_n326), .b(new_n333), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n331), .c(new_n326), .d(new_n330), .o1(\s[31] ));
  aoi022aa1n02x5               g240(.a(new_n100), .b(new_n99), .c(\a[1] ), .d(\b[0] ), .o1(new_n336));
  oaib12aa1n02x5               g241(.a(new_n336), .b(new_n100), .c(\a[2] ), .out0(new_n337));
  norb02aa1n02x5               g242(.a(new_n106), .b(new_n105), .out0(new_n338));
  aboi22aa1n03x5               g243(.a(new_n105), .b(new_n106), .c(new_n99), .d(new_n100), .out0(new_n339));
  aboi22aa1n03x5               g244(.a(new_n102), .b(new_n338), .c(new_n339), .d(new_n337), .out0(\s[3] ));
  norb02aa1n02x5               g245(.a(new_n104), .b(new_n103), .out0(new_n341));
  oaoi03aa1n02x5               g246(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n342));
  inv000aa1d42x5               g247(.a(new_n102), .o1(new_n343));
  aoi112aa1n02x5               g248(.a(new_n105), .b(new_n341), .c(new_n343), .d(new_n106), .o1(new_n344));
  aoi012aa1n02x5               g249(.a(new_n344), .b(new_n341), .c(new_n342), .o1(\s[4] ));
  nanb02aa1n02x5               g250(.a(new_n107), .b(new_n343), .out0(new_n346));
  norb02aa1n02x5               g251(.a(new_n113), .b(new_n112), .out0(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n347), .b(new_n346), .c(new_n108), .out0(\s[5] ));
  norb02aa1n02x5               g253(.a(new_n110), .b(new_n111), .out0(new_n349));
  aoai13aa1n02x5               g254(.a(new_n349), .b(new_n112), .c(new_n109), .d(new_n113), .o1(new_n350));
  aoi112aa1n02x5               g255(.a(new_n112), .b(new_n349), .c(new_n109), .d(new_n347), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n350), .b(new_n351), .out0(\s[6] ));
  inv000aa1d42x5               g257(.a(new_n118), .o1(new_n353));
  oai012aa1n02x5               g258(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n354));
  nanb02aa1n02x5               g259(.a(new_n114), .b(new_n109), .out0(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n353), .b(new_n355), .c(new_n354), .out0(\s[7] ));
  aob012aa1n03x5               g261(.a(new_n353), .b(new_n355), .c(new_n354), .out0(new_n357));
  xobna2aa1n03x5               g262(.a(new_n115), .b(new_n357), .c(new_n120), .out0(\s[8] ));
  norp03aa1n02x5               g263(.a(new_n122), .b(new_n115), .c(new_n123), .o1(new_n359));
  nanp02aa1n02x5               g264(.a(new_n121), .b(new_n148), .o1(new_n360));
  aoi112aa1n02x5               g265(.a(new_n360), .b(new_n359), .c(new_n109), .d(new_n119), .o1(new_n361));
  norb02aa1n02x5               g266(.a(new_n127), .b(new_n361), .out0(\s[9] ));
endmodule


