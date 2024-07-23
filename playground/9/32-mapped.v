// Benchmark "adder" written by ABC on Wed Jul 17 16:50:32 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n339, new_n341, new_n342, new_n345, new_n346, new_n348,
    new_n349, new_n350, new_n352, new_n353, new_n354, new_n355, new_n356,
    new_n357, new_n359, new_n360;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  orn002aa1n02x7               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nand42aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand22aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aob012aa1n03x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nor042aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand42aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor042aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n06x4               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nanp03aa1n09x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  aoi012aa1n09x5               g013(.a(new_n102), .b(new_n105), .c(new_n103), .o1(new_n109));
  nor042aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1n06x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x5               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  nand42aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor002aa1n16x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  norb02aa1n02x7               g021(.a(new_n115), .b(new_n116), .out0(new_n117));
  tech160nm_fixorc02aa1n02p5x5 g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nand23aa1n03x5               g023(.a(new_n114), .b(new_n117), .c(new_n118), .o1(new_n119));
  nano22aa1n02x5               g024(.a(new_n112), .b(new_n111), .c(new_n113), .out0(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  tech160nm_fiaoi012aa1n04x5   g026(.a(new_n110), .b(\a[6] ), .c(\b[5] ), .o1(new_n122));
  tech160nm_fiaoi012aa1n03p5x5 g027(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n123));
  inv000aa1n02x5               g028(.a(new_n123), .o1(new_n124));
  aoi013aa1n06x4               g029(.a(new_n124), .b(new_n120), .c(new_n121), .d(new_n122), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n125), .b(new_n119), .c(new_n108), .d(new_n109), .o1(new_n126));
  tech160nm_fixorc02aa1n05x5   g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  nanp02aa1n06x5               g032(.a(new_n126), .b(new_n127), .o1(new_n128));
  xorc02aa1n12x5               g033(.a(\a[10] ), .b(\b[9] ), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n128), .c(new_n97), .out0(\s[10] ));
  nand42aa1n03x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor022aa1n12x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oai022aa1d18x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  nanb02aa1n06x5               g039(.a(new_n134), .b(new_n128), .out0(new_n135));
  aob012aa1n02x5               g040(.a(new_n135), .b(\b[9] ), .c(\a[10] ), .out0(new_n136));
  aoi022aa1d24x5               g041(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n132), .out0(new_n138));
  aoi022aa1n02x5               g043(.a(new_n136), .b(new_n133), .c(new_n135), .d(new_n138), .o1(\s[11] ));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n10x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n132), .c(new_n135), .d(new_n137), .o1(new_n143));
  aoi112aa1n06x5               g048(.a(new_n142), .b(new_n132), .c(new_n135), .d(new_n137), .o1(new_n144));
  norb02aa1n03x4               g049(.a(new_n143), .b(new_n144), .out0(\s[12] ));
  nano23aa1n03x7               g050(.a(new_n140), .b(new_n132), .c(new_n141), .d(new_n131), .out0(new_n146));
  nand23aa1n04x5               g051(.a(new_n146), .b(new_n127), .c(new_n129), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nand22aa1n03x5               g053(.a(new_n126), .b(new_n148), .o1(new_n149));
  oai022aa1n06x5               g054(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n150));
  aoai13aa1n12x5               g055(.a(new_n141), .b(new_n150), .c(new_n134), .d(new_n137), .o1(new_n151));
  nanp02aa1n03x5               g056(.a(new_n149), .b(new_n151), .o1(new_n152));
  tech160nm_fixorc02aa1n04x5   g057(.a(\a[13] ), .b(\b[12] ), .out0(new_n153));
  nano32aa1n02x4               g058(.a(new_n150), .b(new_n134), .c(new_n137), .d(new_n141), .out0(new_n154));
  aoi112aa1n02x5               g059(.a(new_n154), .b(new_n153), .c(new_n141), .d(new_n150), .o1(new_n155));
  aoi022aa1n02x5               g060(.a(new_n152), .b(new_n153), .c(new_n149), .d(new_n155), .o1(\s[13] ));
  nor042aa1n06x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n151), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n153), .b(new_n159), .c(new_n126), .d(new_n148), .o1(new_n160));
  tech160nm_fixorc02aa1n02p5x5 g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n160), .c(new_n158), .out0(\s[14] ));
  nand02aa1d06x5               g067(.a(new_n161), .b(new_n153), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n159), .c(new_n126), .d(new_n148), .o1(new_n165));
  oaoi03aa1n09x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  norp02aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n165), .c(new_n167), .out0(\s[15] ));
  aoai13aa1n03x5               g076(.a(new_n170), .b(new_n166), .c(new_n152), .d(new_n164), .o1(new_n172));
  inv000aa1n02x5               g077(.a(new_n168), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n170), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n174), .c(new_n165), .d(new_n167), .o1(new_n175));
  nor002aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n04x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  norp02aa1n02x5               g083(.a(new_n178), .b(new_n168), .o1(new_n179));
  aoi022aa1n03x5               g084(.a(new_n175), .b(new_n178), .c(new_n172), .d(new_n179), .o1(\s[16] ));
  nona23aa1n09x5               g085(.a(new_n177), .b(new_n169), .c(new_n168), .d(new_n176), .out0(new_n181));
  nor043aa1d12x5               g086(.a(new_n147), .b(new_n163), .c(new_n181), .o1(new_n182));
  nanp02aa1n09x5               g087(.a(new_n126), .b(new_n182), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(\b[13] ), .b(\a[14] ), .o1(new_n184));
  oai022aa1n02x5               g089(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n185));
  nanb03aa1n03x5               g090(.a(new_n176), .b(new_n177), .c(new_n169), .out0(new_n186));
  nano32aa1n03x5               g091(.a(new_n186), .b(new_n185), .c(new_n173), .d(new_n184), .out0(new_n187));
  aoi012aa1n02x5               g092(.a(new_n176), .b(new_n168), .c(new_n177), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n188), .b(new_n187), .out0(new_n189));
  oai013aa1d12x5               g094(.a(new_n189), .b(new_n163), .c(new_n151), .d(new_n181), .o1(new_n190));
  nanb02aa1n12x5               g095(.a(new_n190), .b(new_n183), .out0(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  nona22aa1n02x4               g097(.a(new_n164), .b(new_n151), .c(new_n181), .out0(new_n193));
  nano23aa1n02x4               g098(.a(new_n192), .b(new_n187), .c(new_n193), .d(new_n188), .out0(new_n194));
  aoi022aa1n02x5               g099(.a(new_n191), .b(new_n192), .c(new_n194), .d(new_n183), .o1(\s[17] ));
  nor042aa1n09x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n192), .b(new_n190), .c(new_n126), .d(new_n182), .o1(new_n198));
  nor042aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand02aa1n03x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  norb02aa1n06x4               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n197), .out0(\s[18] ));
  and002aa1n02x5               g107(.a(new_n192), .b(new_n201), .o(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n190), .c(new_n126), .d(new_n182), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  nor042aa1d18x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nanp02aa1n04x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1n12x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n02x5               g116(.a(new_n209), .b(new_n204), .c(new_n206), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n207), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n209), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n213), .b(new_n214), .c(new_n204), .d(new_n206), .o1(new_n215));
  nor042aa1n03x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1n08x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  inv000aa1d42x5               g123(.a(\a[19] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\b[18] ), .o1(new_n220));
  aboi22aa1n03x5               g125(.a(new_n216), .b(new_n217), .c(new_n219), .d(new_n220), .out0(new_n221));
  aoi022aa1n02x5               g126(.a(new_n215), .b(new_n218), .c(new_n212), .d(new_n221), .o1(\s[20] ));
  nano32aa1n03x7               g127(.a(new_n214), .b(new_n192), .c(new_n218), .d(new_n201), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n190), .c(new_n126), .d(new_n182), .o1(new_n224));
  nanb03aa1n12x5               g129(.a(new_n216), .b(new_n217), .c(new_n208), .out0(new_n225));
  oai112aa1n06x5               g130(.a(new_n213), .b(new_n200), .c(new_n199), .d(new_n196), .o1(new_n226));
  aoi012aa1n06x5               g131(.a(new_n216), .b(new_n207), .c(new_n217), .o1(new_n227));
  oai012aa1n18x5               g132(.a(new_n227), .b(new_n226), .c(new_n225), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  nor002aa1n16x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  nanp02aa1n03x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  norb02aa1d27x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  aob012aa1n03x5               g137(.a(new_n232), .b(new_n224), .c(new_n229), .out0(new_n233));
  nano22aa1n03x5               g138(.a(new_n216), .b(new_n208), .c(new_n217), .out0(new_n234));
  oai012aa1n02x5               g139(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .o1(new_n235));
  oab012aa1n02x5               g140(.a(new_n235), .b(new_n196), .c(new_n199), .out0(new_n236));
  inv000aa1n03x5               g141(.a(new_n227), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n237), .b(new_n232), .c(new_n236), .d(new_n234), .o1(new_n238));
  aobi12aa1n02x7               g143(.a(new_n233), .b(new_n238), .c(new_n224), .out0(\s[21] ));
  inv000aa1d42x5               g144(.a(new_n230), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n232), .o1(new_n241));
  aoai13aa1n02x5               g146(.a(new_n240), .b(new_n241), .c(new_n224), .d(new_n229), .o1(new_n242));
  nor042aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoib12aa1n02x5               g150(.a(new_n230), .b(new_n244), .c(new_n243), .out0(new_n246));
  aoi022aa1n02x7               g151(.a(new_n242), .b(new_n245), .c(new_n233), .d(new_n246), .o1(\s[22] ));
  inv000aa1n02x5               g152(.a(new_n223), .o1(new_n248));
  nano22aa1n03x7               g153(.a(new_n248), .b(new_n232), .c(new_n245), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n190), .c(new_n126), .d(new_n182), .o1(new_n250));
  nano23aa1n06x5               g155(.a(new_n230), .b(new_n243), .c(new_n244), .d(new_n231), .out0(new_n251));
  aoi012aa1n02x5               g156(.a(new_n243), .b(new_n230), .c(new_n244), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n228), .c(new_n251), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aob012aa1n03x5               g160(.a(new_n255), .b(new_n250), .c(new_n254), .out0(new_n256));
  aoi112aa1n02x5               g161(.a(new_n255), .b(new_n253), .c(new_n228), .d(new_n251), .o1(new_n257));
  aobi12aa1n02x7               g162(.a(new_n256), .b(new_n257), .c(new_n250), .out0(\s[23] ));
  nor042aa1n06x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n255), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n260), .b(new_n261), .c(new_n250), .d(new_n254), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .out0(new_n263));
  norp02aa1n02x5               g168(.a(new_n263), .b(new_n259), .o1(new_n264));
  aoi022aa1n03x5               g169(.a(new_n262), .b(new_n263), .c(new_n256), .d(new_n264), .o1(\s[24] ));
  and002aa1n02x5               g170(.a(new_n263), .b(new_n255), .o(new_n266));
  nano22aa1n02x4               g171(.a(new_n248), .b(new_n266), .c(new_n251), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n190), .c(new_n126), .d(new_n182), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n251), .b(new_n237), .c(new_n236), .d(new_n234), .o1(new_n269));
  inv000aa1n03x5               g174(.a(new_n266), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[24] ), .b(\b[23] ), .c(new_n260), .carry(new_n271));
  aoai13aa1n12x5               g176(.a(new_n271), .b(new_n270), .c(new_n269), .d(new_n252), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  aob012aa1n03x5               g179(.a(new_n274), .b(new_n268), .c(new_n273), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n266), .b(new_n253), .c(new_n228), .d(new_n251), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n274), .o1(new_n277));
  and003aa1n02x5               g182(.a(new_n276), .b(new_n277), .c(new_n271), .o(new_n278));
  aobi12aa1n02x7               g183(.a(new_n275), .b(new_n278), .c(new_n268), .out0(\s[25] ));
  nor042aa1n03x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n281), .b(new_n277), .c(new_n268), .d(new_n273), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n280), .o1(new_n284));
  aoi022aa1n03x5               g189(.a(new_n282), .b(new_n283), .c(new_n275), .d(new_n284), .o1(\s[26] ));
  and002aa1n18x5               g190(.a(new_n283), .b(new_n274), .o(new_n286));
  nano32aa1n03x7               g191(.a(new_n248), .b(new_n286), .c(new_n251), .d(new_n266), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n190), .c(new_n126), .d(new_n182), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n286), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .carry(new_n290));
  aoai13aa1n04x5               g195(.a(new_n290), .b(new_n289), .c(new_n276), .d(new_n271), .o1(new_n291));
  xorc02aa1n12x5               g196(.a(\a[27] ), .b(\b[26] ), .out0(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n291), .c(new_n191), .d(new_n287), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n290), .o1(new_n294));
  aoi112aa1n02x5               g199(.a(new_n292), .b(new_n294), .c(new_n272), .d(new_n286), .o1(new_n295));
  aobi12aa1n03x7               g200(.a(new_n293), .b(new_n295), .c(new_n288), .out0(\s[27] ));
  tech160nm_fiaoi012aa1n05x5   g201(.a(new_n294), .b(new_n272), .c(new_n286), .o1(new_n297));
  nor042aa1n06x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n292), .o1(new_n300));
  aoai13aa1n02x7               g205(.a(new_n299), .b(new_n300), .c(new_n297), .d(new_n288), .o1(new_n301));
  xorc02aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .out0(new_n302));
  norp02aa1n02x5               g207(.a(new_n302), .b(new_n298), .o1(new_n303));
  aoi022aa1n02x7               g208(.a(new_n301), .b(new_n302), .c(new_n293), .d(new_n303), .o1(\s[28] ));
  inv000aa1d42x5               g209(.a(\a[27] ), .o1(new_n305));
  inv000aa1d42x5               g210(.a(\a[28] ), .o1(new_n306));
  xroi22aa1d04x5               g211(.a(new_n305), .b(\b[26] ), .c(new_n306), .d(\b[27] ), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n291), .c(new_n191), .d(new_n287), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n307), .o1(new_n309));
  inv000aa1d42x5               g214(.a(\b[27] ), .o1(new_n310));
  oaoi03aa1n02x5               g215(.a(new_n306), .b(new_n310), .c(new_n298), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n309), .c(new_n297), .d(new_n288), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n308), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g221(.a(new_n300), .b(new_n302), .c(new_n313), .out0(new_n317));
  aoai13aa1n06x5               g222(.a(new_n317), .b(new_n291), .c(new_n191), .d(new_n287), .o1(new_n318));
  inv000aa1n02x5               g223(.a(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(\a[29] ), .o1(new_n320));
  inv000aa1d42x5               g225(.a(\b[28] ), .o1(new_n321));
  aoi012aa1n02x5               g226(.a(new_n298), .b(new_n306), .c(new_n310), .o1(new_n322));
  aoi122aa1n02x5               g227(.a(new_n322), .b(\b[28] ), .c(\a[29] ), .d(\b[27] ), .e(\a[28] ), .o1(new_n323));
  aoi012aa1n02x5               g228(.a(new_n323), .b(new_n320), .c(new_n321), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n319), .c(new_n297), .d(new_n288), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .out0(new_n326));
  aoi112aa1n02x5               g231(.a(new_n323), .b(new_n326), .c(new_n320), .d(new_n321), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n325), .b(new_n326), .c(new_n318), .d(new_n327), .o1(\s[30] ));
  nano32aa1n06x5               g233(.a(new_n300), .b(new_n326), .c(new_n302), .d(new_n313), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n291), .c(new_n191), .d(new_n287), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  nanp02aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .o1(new_n332));
  oai022aa1n02x5               g237(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n333));
  oaoi13aa1n02x5               g238(.a(new_n331), .b(new_n332), .c(new_n323), .d(new_n333), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n329), .o1(new_n335));
  oai012aa1n02x5               g240(.a(new_n332), .b(new_n323), .c(new_n333), .o1(new_n336));
  aoai13aa1n02x7               g241(.a(new_n336), .b(new_n335), .c(new_n297), .d(new_n288), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n331), .c(new_n330), .d(new_n334), .o1(\s[31] ));
  nanp03aa1n02x5               g243(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n339));
  xnbna2aa1n03x5               g244(.a(new_n107), .b(new_n339), .c(new_n98), .out0(\s[3] ));
  nanp02aa1n02x5               g245(.a(new_n108), .b(new_n109), .o1(new_n341));
  aoi112aa1n02x5               g246(.a(new_n105), .b(new_n104), .c(new_n101), .d(new_n107), .o1(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xnbna2aa1n03x5               g248(.a(new_n118), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  orn002aa1n02x5               g249(.a(\a[5] ), .b(\b[4] ), .o(new_n345));
  nanp02aa1n02x5               g250(.a(new_n341), .b(new_n118), .o1(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n117), .b(new_n346), .c(new_n345), .out0(\s[6] ));
  norb02aa1n02x5               g252(.a(new_n113), .b(new_n112), .out0(new_n348));
  inv000aa1d42x5               g253(.a(new_n116), .o1(new_n349));
  aob012aa1n02x5               g254(.a(new_n117), .b(new_n346), .c(new_n345), .out0(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n348), .b(new_n350), .c(new_n349), .out0(\s[7] ));
  norb02aa1n02x5               g256(.a(new_n111), .b(new_n110), .out0(new_n352));
  aob012aa1n02x5               g257(.a(new_n348), .b(new_n350), .c(new_n349), .out0(new_n353));
  inv000aa1d42x5               g258(.a(new_n112), .o1(new_n354));
  inv000aa1d42x5               g259(.a(new_n348), .o1(new_n355));
  aoai13aa1n02x5               g260(.a(new_n354), .b(new_n355), .c(new_n350), .d(new_n349), .o1(new_n356));
  aoib12aa1n02x5               g261(.a(new_n112), .b(new_n111), .c(new_n110), .out0(new_n357));
  aoi022aa1n02x5               g262(.a(new_n356), .b(new_n352), .c(new_n353), .d(new_n357), .o1(\s[8] ));
  aoi012aa1n02x5               g263(.a(new_n119), .b(new_n108), .c(new_n109), .o1(new_n359));
  aoi113aa1n02x5               g264(.a(new_n124), .b(new_n127), .c(new_n120), .d(new_n121), .e(new_n122), .o1(new_n360));
  aboi22aa1n03x5               g265(.a(new_n359), .b(new_n360), .c(new_n126), .d(new_n127), .out0(\s[9] ));
endmodule


