// Benchmark "adder" written by ABC on Thu Jul 18 12:29:19 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n349, new_n350, new_n351,
    new_n353, new_n355, new_n357, new_n358, new_n359, new_n360, new_n362,
    new_n363, new_n364, new_n366, new_n368, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[1] ), .o1(new_n102));
  inv040aa1d28x5               g007(.a(\b[0] ), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  oaoi13aa1n06x5               g010(.a(new_n104), .b(new_n105), .c(new_n102), .d(new_n103), .o1(new_n106));
  nor002aa1n16x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nand42aa1n16x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nor002aa1d32x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nand02aa1d04x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nona23aa1n09x5               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  tech160nm_fioai012aa1n04x5   g016(.a(new_n108), .b(new_n109), .c(new_n107), .o1(new_n112));
  oaih12aa1n06x5               g017(.a(new_n112), .b(new_n111), .c(new_n106), .o1(new_n113));
  nor002aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nor002aa1n03x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n116), .b(new_n115), .c(new_n117), .d(new_n114), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  nor043aa1n03x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  oai022aa1n02x7               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  aoi022aa1d24x5               g028(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n116), .b(new_n114), .c(new_n123), .d(new_n124), .o1(new_n125));
  oaib12aa1n09x5               g030(.a(new_n125), .b(\b[7] ), .c(new_n122), .out0(new_n126));
  nand42aa1n08x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n100), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n126), .c(new_n113), .d(new_n121), .o1(new_n129));
  xobna2aa1n03x5               g034(.a(new_n99), .b(new_n129), .c(new_n101), .out0(\s[10] ));
  nor042aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  norp02aa1n03x5               g038(.a(new_n100), .b(new_n97), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n129), .b(new_n134), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n133), .b(new_n135), .c(new_n98), .out0(\s[11] ));
  nor042aa1n04x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  nano22aa1n02x4               g044(.a(new_n131), .b(new_n98), .c(new_n132), .out0(new_n140));
  aoi012aa1n02x5               g045(.a(new_n131), .b(new_n135), .c(new_n140), .o1(new_n141));
  norb03aa1n03x5               g046(.a(new_n138), .b(new_n131), .c(new_n137), .out0(new_n142));
  aob012aa1n02x5               g047(.a(new_n142), .b(new_n135), .c(new_n140), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n143), .b(new_n141), .c(new_n139), .o1(\s[12] ));
  nano23aa1n09x5               g049(.a(new_n97), .b(new_n137), .c(new_n138), .d(new_n98), .out0(new_n145));
  nano23aa1n09x5               g050(.a(new_n100), .b(new_n131), .c(new_n132), .d(new_n127), .out0(new_n146));
  nand22aa1n06x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n126), .c(new_n113), .d(new_n121), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n132), .b(new_n98), .o1(new_n150));
  tech160nm_fioai012aa1n04x5   g055(.a(new_n142), .b(new_n134), .c(new_n150), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n138), .o1(new_n152));
  nor002aa1n16x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  tech160nm_finand02aa1n05x5   g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n149), .c(new_n152), .out0(\s[13] ));
  inv000aa1n02x5               g061(.a(new_n153), .o1(new_n157));
  nanp02aa1n06x5               g062(.a(new_n149), .b(new_n152), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n158), .b(new_n155), .o1(new_n159));
  norp02aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n09x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  oai022aa1d18x5               g067(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n163));
  nanb03aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n161), .out0(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n162), .c(new_n157), .d(new_n159), .o1(\s[14] ));
  nano23aa1n06x5               g070(.a(new_n153), .b(new_n160), .c(new_n161), .d(new_n154), .out0(new_n166));
  oaoi03aa1n02x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n167));
  tech160nm_fixorc02aa1n03p5x5 g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n167), .c(new_n158), .d(new_n166), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n168), .b(new_n167), .c(new_n158), .d(new_n166), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  inv000aa1d42x5               g076(.a(\a[15] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\b[14] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n173), .b(new_n172), .o1(new_n174));
  norp02aa1n04x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  and002aa1n03x5               g080(.a(\b[15] ), .b(\a[16] ), .o(new_n176));
  nor002aa1n02x5               g081(.a(new_n176), .b(new_n175), .o1(new_n177));
  oai022aa1n02x5               g082(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n178));
  nona22aa1n02x4               g083(.a(new_n169), .b(new_n176), .c(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n174), .d(new_n169), .o1(\s[16] ));
  nano32aa1n03x7               g085(.a(new_n147), .b(new_n177), .c(new_n166), .d(new_n168), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n126), .c(new_n113), .d(new_n121), .o1(new_n182));
  aoi112aa1n03x5               g087(.a(new_n176), .b(new_n175), .c(\a[15] ), .d(\b[14] ), .o1(new_n183));
  oai112aa1n02x7               g088(.a(new_n154), .b(new_n161), .c(\b[14] ), .d(\a[15] ), .o1(new_n184));
  nano23aa1n03x7               g089(.a(new_n184), .b(new_n160), .c(new_n157), .d(new_n138), .out0(new_n185));
  oai112aa1n03x5               g090(.a(new_n163), .b(new_n161), .c(new_n173), .d(new_n172), .o1(new_n186));
  aoi022aa1n02x5               g091(.a(new_n186), .b(new_n174), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  aoi113aa1n09x5               g092(.a(new_n187), .b(new_n175), .c(new_n185), .d(new_n151), .e(new_n183), .o1(new_n188));
  nanp02aa1n09x5               g093(.a(new_n182), .b(new_n188), .o1(new_n189));
  nor042aa1n09x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  nand22aa1n03x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  obai22aa1n02x7               g097(.a(new_n191), .b(new_n190), .c(\a[16] ), .d(\b[15] ), .out0(new_n193));
  aoi113aa1n02x5               g098(.a(new_n193), .b(new_n187), .c(new_n185), .d(new_n151), .e(new_n183), .o1(new_n194));
  aoi022aa1n02x5               g099(.a(new_n189), .b(new_n192), .c(new_n182), .d(new_n194), .o1(\s[17] ));
  inv000aa1d42x5               g100(.a(new_n190), .o1(new_n196));
  and002aa1n12x5               g101(.a(\b[0] ), .b(\a[1] ), .o(new_n197));
  oaoi03aa1n03x5               g102(.a(\a[2] ), .b(\b[1] ), .c(new_n197), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n108), .b(new_n107), .out0(new_n199));
  norb02aa1n06x5               g104(.a(new_n110), .b(new_n109), .out0(new_n200));
  nand23aa1n03x5               g105(.a(new_n198), .b(new_n199), .c(new_n200), .o1(new_n201));
  nano23aa1n02x4               g106(.a(new_n117), .b(new_n114), .c(new_n115), .d(new_n116), .out0(new_n202));
  nona22aa1n03x5               g107(.a(new_n202), .b(new_n119), .c(new_n120), .out0(new_n203));
  nor022aa1n04x5               g108(.a(\b[4] ), .b(\a[5] ), .o1(new_n204));
  nor002aa1n02x5               g109(.a(\b[5] ), .b(\a[6] ), .o1(new_n205));
  tech160nm_fioai012aa1n05x5   g110(.a(new_n124), .b(new_n205), .c(new_n204), .o1(new_n206));
  oai012aa1n02x5               g111(.a(new_n206), .b(\b[6] ), .c(\a[7] ), .o1(new_n207));
  tech160nm_fiaoi012aa1n02p5x5 g112(.a(new_n117), .b(new_n207), .c(new_n116), .o1(new_n208));
  aoai13aa1n04x5               g113(.a(new_n208), .b(new_n203), .c(new_n201), .d(new_n112), .o1(new_n209));
  nanp03aa1n02x5               g114(.a(new_n185), .b(new_n151), .c(new_n183), .o1(new_n210));
  nona22aa1n02x4               g115(.a(new_n210), .b(new_n187), .c(new_n175), .out0(new_n211));
  aoai13aa1n02x5               g116(.a(new_n192), .b(new_n211), .c(new_n209), .d(new_n181), .o1(new_n212));
  nor042aa1n02x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nand22aa1n04x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  oai022aa1n04x5               g120(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n216));
  nanb03aa1n02x5               g121(.a(new_n216), .b(new_n212), .c(new_n214), .out0(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n215), .c(new_n196), .d(new_n212), .o1(\s[18] ));
  nano23aa1n06x5               g123(.a(new_n190), .b(new_n213), .c(new_n214), .d(new_n191), .out0(new_n219));
  oaoi03aa1n02x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n220));
  norp02aa1n24x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand42aa1n02x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  norb02aa1n06x4               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n220), .c(new_n189), .d(new_n219), .o1(new_n224));
  aoi112aa1n02x7               g129(.a(new_n223), .b(new_n220), .c(new_n189), .d(new_n219), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n224), .b(new_n225), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g132(.a(new_n221), .o1(new_n228));
  nor042aa1n03x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  nanp02aa1n04x5               g134(.a(\b[19] ), .b(\a[20] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  norb03aa1n02x5               g136(.a(new_n230), .b(new_n221), .c(new_n229), .out0(new_n232));
  nanp02aa1n03x5               g137(.a(new_n224), .b(new_n232), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n231), .c(new_n228), .d(new_n224), .o1(\s[20] ));
  nand03aa1n02x5               g139(.a(new_n219), .b(new_n223), .c(new_n231), .o1(new_n235));
  inv040aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  nanb03aa1n02x5               g141(.a(new_n229), .b(new_n230), .c(new_n222), .out0(new_n237));
  oai112aa1n02x5               g142(.a(new_n216), .b(new_n214), .c(\b[18] ), .d(\a[19] ), .o1(new_n238));
  tech160nm_fioai012aa1n03p5x5 g143(.a(new_n230), .b(new_n229), .c(new_n221), .o1(new_n239));
  tech160nm_fioai012aa1n03p5x5 g144(.a(new_n239), .b(new_n238), .c(new_n237), .o1(new_n240));
  nor002aa1d32x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  nand02aa1n06x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n240), .c(new_n189), .d(new_n236), .o1(new_n244));
  nano22aa1n03x5               g149(.a(new_n229), .b(new_n222), .c(new_n230), .out0(new_n245));
  oai012aa1n02x5               g150(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .o1(new_n246));
  oab012aa1n04x5               g151(.a(new_n246), .b(new_n190), .c(new_n213), .out0(new_n247));
  inv040aa1n02x5               g152(.a(new_n239), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n248), .b(new_n243), .c(new_n247), .d(new_n245), .o1(new_n249));
  aobi12aa1n02x5               g154(.a(new_n249), .b(new_n189), .c(new_n236), .out0(new_n250));
  norb02aa1n03x4               g155(.a(new_n244), .b(new_n250), .out0(\s[21] ));
  inv000aa1d42x5               g156(.a(new_n241), .o1(new_n252));
  nor042aa1n03x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  nand42aa1n06x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  norb03aa1n02x5               g160(.a(new_n254), .b(new_n241), .c(new_n253), .out0(new_n256));
  nanp02aa1n03x5               g161(.a(new_n244), .b(new_n256), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n252), .d(new_n244), .o1(\s[22] ));
  nano23aa1d15x5               g163(.a(new_n241), .b(new_n253), .c(new_n254), .d(new_n242), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  nano32aa1n02x4               g165(.a(new_n260), .b(new_n219), .c(new_n223), .d(new_n231), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n259), .b(new_n248), .c(new_n247), .d(new_n245), .o1(new_n262));
  oaoi03aa1n12x5               g167(.a(\a[22] ), .b(\b[21] ), .c(new_n252), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(new_n262), .b(new_n264), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[23] ), .b(\b[22] ), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n265), .c(new_n189), .d(new_n261), .o1(new_n267));
  nona22aa1n02x4               g172(.a(new_n262), .b(new_n263), .c(new_n266), .out0(new_n268));
  aoi012aa1n02x5               g173(.a(new_n268), .b(new_n189), .c(new_n261), .o1(new_n269));
  norb02aa1n03x4               g174(.a(new_n267), .b(new_n269), .out0(\s[23] ));
  norp02aa1n02x5               g175(.a(\b[22] ), .b(\a[23] ), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  tech160nm_fixorc02aa1n05x5   g177(.a(\a[24] ), .b(\b[23] ), .out0(new_n273));
  oai022aa1n02x5               g178(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n274));
  aoi012aa1n02x5               g179(.a(new_n274), .b(\a[24] ), .c(\b[23] ), .o1(new_n275));
  nanp02aa1n03x5               g180(.a(new_n267), .b(new_n275), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n273), .c(new_n272), .d(new_n267), .o1(\s[24] ));
  nano32aa1n02x5               g182(.a(new_n235), .b(new_n273), .c(new_n259), .d(new_n266), .out0(new_n278));
  nanp02aa1n02x5               g183(.a(new_n273), .b(new_n266), .o1(new_n279));
  aob012aa1n02x5               g184(.a(new_n274), .b(\b[23] ), .c(\a[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n262), .d(new_n264), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n189), .d(new_n278), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n279), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n263), .c(new_n240), .d(new_n259), .o1(new_n285));
  nanb03aa1n02x5               g190(.a(new_n282), .b(new_n285), .c(new_n280), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n189), .c(new_n278), .o1(new_n287));
  norb02aa1n03x4               g192(.a(new_n283), .b(new_n287), .out0(\s[25] ));
  nor042aa1n03x5               g193(.a(\b[24] ), .b(\a[25] ), .o1(new_n289));
  inv000aa1n03x5               g194(.a(new_n289), .o1(new_n290));
  xorc02aa1n06x5               g195(.a(\a[26] ), .b(\b[25] ), .out0(new_n291));
  nanp02aa1n02x5               g196(.a(\b[25] ), .b(\a[26] ), .o1(new_n292));
  oai022aa1n02x5               g197(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n293));
  norb02aa1n02x5               g198(.a(new_n292), .b(new_n293), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n283), .b(new_n294), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n290), .d(new_n283), .o1(\s[26] ));
  nand02aa1n04x5               g201(.a(new_n291), .b(new_n282), .o1(new_n297));
  nano23aa1n03x7               g202(.a(new_n235), .b(new_n297), .c(new_n284), .d(new_n259), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n211), .c(new_n209), .d(new_n181), .o1(new_n299));
  nona32aa1n03x5               g204(.a(new_n236), .b(new_n297), .c(new_n279), .d(new_n260), .out0(new_n300));
  aoi012aa1n06x5               g205(.a(new_n300), .b(new_n182), .c(new_n188), .o1(new_n301));
  oaoi03aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n290), .o1(new_n302));
  inv000aa1n02x5               g207(.a(new_n302), .o1(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n297), .c(new_n285), .d(new_n280), .o1(new_n304));
  xorc02aa1n03x5               g209(.a(\a[27] ), .b(\b[26] ), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n305), .b(new_n304), .c(new_n301), .o1(new_n306));
  inv000aa1d42x5               g211(.a(new_n297), .o1(new_n307));
  aoi112aa1n02x5               g212(.a(new_n305), .b(new_n302), .c(new_n281), .d(new_n307), .o1(new_n308));
  aobi12aa1n02x7               g213(.a(new_n306), .b(new_n308), .c(new_n299), .out0(\s[27] ));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n310), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .out0(new_n312));
  aoi022aa1n06x5               g217(.a(new_n281), .b(new_n307), .c(new_n292), .d(new_n293), .o1(new_n313));
  inv000aa1n03x5               g218(.a(new_n305), .o1(new_n314));
  oai022aa1n02x5               g219(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n315), .b(\a[28] ), .c(\b[27] ), .o1(new_n316));
  aoai13aa1n04x5               g221(.a(new_n316), .b(new_n314), .c(new_n313), .d(new_n299), .o1(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n312), .c(new_n306), .d(new_n311), .o1(\s[28] ));
  and002aa1n02x5               g223(.a(new_n312), .b(new_n305), .o(new_n319));
  oaih12aa1n02x5               g224(.a(new_n319), .b(new_n304), .c(new_n301), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n319), .o1(new_n321));
  aob012aa1n02x5               g226(.a(new_n315), .b(\b[27] ), .c(\a[28] ), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n321), .c(new_n313), .d(new_n299), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .out0(new_n324));
  norb02aa1n02x5               g229(.a(new_n322), .b(new_n324), .out0(new_n325));
  aoi022aa1n03x5               g230(.a(new_n323), .b(new_n324), .c(new_n320), .d(new_n325), .o1(\s[29] ));
  xnrb03aa1n02x5               g231(.a(new_n197), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g232(.a(new_n314), .b(new_n312), .c(new_n324), .out0(new_n328));
  oai012aa1n02x5               g233(.a(new_n328), .b(new_n304), .c(new_n301), .o1(new_n329));
  inv000aa1n02x5               g234(.a(new_n328), .o1(new_n330));
  inv000aa1d42x5               g235(.a(\a[29] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[28] ), .o1(new_n332));
  aoi022aa1n02x5               g237(.a(\b[28] ), .b(\a[29] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n333));
  aoi022aa1n02x5               g238(.a(new_n315), .b(new_n333), .c(new_n332), .d(new_n331), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n330), .c(new_n313), .d(new_n299), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[30] ), .b(\b[29] ), .out0(new_n336));
  aoi122aa1n02x5               g241(.a(new_n336), .b(new_n315), .c(new_n333), .d(new_n331), .e(new_n332), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n335), .b(new_n336), .c(new_n329), .d(new_n337), .o1(\s[30] ));
  nano32aa1n02x4               g243(.a(new_n314), .b(new_n336), .c(new_n312), .d(new_n324), .out0(new_n339));
  oaih12aa1n02x5               g244(.a(new_n339), .b(new_n304), .c(new_n301), .o1(new_n340));
  inv000aa1n02x5               g245(.a(new_n339), .o1(new_n341));
  oao003aa1n02x5               g246(.a(\a[30] ), .b(\b[29] ), .c(new_n334), .carry(new_n342));
  aoai13aa1n03x5               g247(.a(new_n342), .b(new_n341), .c(new_n313), .d(new_n299), .o1(new_n343));
  xorc02aa1n02x5               g248(.a(\a[31] ), .b(\b[30] ), .out0(new_n344));
  nanp02aa1n02x5               g249(.a(\b[29] ), .b(\a[30] ), .o1(new_n345));
  oabi12aa1n02x5               g250(.a(new_n344), .b(\a[30] ), .c(\b[29] ), .out0(new_n346));
  aoib12aa1n02x5               g251(.a(new_n346), .b(new_n345), .c(new_n334), .out0(new_n347));
  aoi022aa1n03x5               g252(.a(new_n343), .b(new_n344), .c(new_n340), .d(new_n347), .o1(\s[31] ));
  norb03aa1n02x5               g253(.a(new_n105), .b(new_n197), .c(new_n104), .out0(new_n349));
  inv000aa1d42x5               g254(.a(new_n109), .o1(new_n350));
  aoi012aa1n02x5               g255(.a(new_n104), .b(new_n350), .c(new_n110), .o1(new_n351));
  aboi22aa1n03x5               g256(.a(new_n349), .b(new_n351), .c(new_n198), .d(new_n200), .out0(\s[3] ));
  nanp02aa1n02x5               g257(.a(new_n198), .b(new_n200), .o1(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n199), .b(new_n353), .c(new_n350), .out0(\s[4] ));
  inv000aa1d42x5               g259(.a(new_n119), .o1(new_n355));
  xnbna2aa1n03x5               g260(.a(new_n355), .b(new_n201), .c(new_n112), .out0(\s[5] ));
  aoai13aa1n02x5               g261(.a(new_n120), .b(new_n204), .c(new_n113), .d(new_n355), .o1(new_n357));
  and002aa1n02x5               g262(.a(\b[5] ), .b(\a[6] ), .o(new_n358));
  nanp02aa1n02x5               g263(.a(new_n113), .b(new_n355), .o1(new_n359));
  nona32aa1n02x4               g264(.a(new_n359), .b(new_n358), .c(new_n205), .d(new_n204), .out0(new_n360));
  nanp02aa1n02x5               g265(.a(new_n360), .b(new_n357), .o1(\s[6] ));
  nanb02aa1n02x5               g266(.a(new_n114), .b(new_n115), .out0(new_n362));
  nanb02aa1n02x5               g267(.a(new_n358), .b(new_n360), .out0(new_n363));
  norb03aa1n02x5               g268(.a(new_n115), .b(new_n358), .c(new_n114), .out0(new_n364));
  aoi022aa1n02x5               g269(.a(new_n363), .b(new_n362), .c(new_n360), .d(new_n364), .o1(\s[7] ));
  aoi012aa1n02x5               g270(.a(new_n114), .b(new_n360), .c(new_n124), .o1(new_n366));
  xorb03aa1n02x5               g271(.a(new_n366), .b(\b[7] ), .c(new_n122), .out0(\s[8] ));
  nanp02aa1n02x5               g272(.a(new_n113), .b(new_n121), .o1(new_n368));
  aoi112aa1n02x5               g273(.a(new_n128), .b(new_n117), .c(new_n207), .d(new_n116), .o1(new_n369));
  aoi022aa1n02x5               g274(.a(new_n209), .b(new_n128), .c(new_n368), .d(new_n369), .o1(\s[9] ));
endmodule

