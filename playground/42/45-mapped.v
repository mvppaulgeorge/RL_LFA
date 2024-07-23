// Benchmark "adder" written by ABC on Thu Jul 18 09:54:09 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n344,
    new_n346, new_n347, new_n348, new_n349, new_n350, new_n353, new_n354,
    new_n355, new_n356, new_n358, new_n359, new_n360, new_n362, new_n363;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n10x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  oa0022aa1n06x5               g002(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n98));
  tech160nm_fixnrc02aa1n04x5   g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  nand42aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d16x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  tech160nm_fioai012aa1n05x5   g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  orn002aa1n02x5               g008(.a(new_n99), .b(new_n103), .o(new_n104));
  nor042aa1n04x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  and002aa1n12x5               g010(.a(\b[5] ), .b(\a[6] ), .o(new_n106));
  aoi112aa1n09x5               g011(.a(new_n106), .b(new_n105), .c(\a[4] ), .d(\b[3] ), .o1(new_n107));
  xorc02aa1n12x5               g012(.a(\a[8] ), .b(\b[7] ), .out0(new_n108));
  nor002aa1n16x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nanp02aa1n04x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand02aa1d28x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n112), .b(new_n109), .c(new_n110), .d(new_n111), .out0(new_n113));
  nand23aa1n06x5               g018(.a(new_n113), .b(new_n107), .c(new_n108), .o1(new_n114));
  orn002aa1n02x5               g019(.a(\a[8] ), .b(\b[7] ), .o(new_n115));
  inv040aa1n02x5               g020(.a(new_n112), .o1(new_n116));
  oai022aa1d18x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  aob012aa1n12x5               g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .out0(new_n118));
  aob012aa1n02x5               g023(.a(new_n111), .b(\b[7] ), .c(\a[8] ), .out0(new_n119));
  aoi012aa1n02x5               g024(.a(new_n119), .b(new_n118), .c(new_n116), .o1(new_n120));
  nor042aa1n09x5               g025(.a(\b[8] ), .b(\a[9] ), .o1(new_n121));
  norb02aa1n02x5               g026(.a(new_n97), .b(new_n121), .out0(new_n122));
  nano22aa1n03x7               g027(.a(new_n120), .b(new_n115), .c(new_n122), .out0(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n114), .c(new_n98), .d(new_n104), .o1(new_n124));
  nor042aa1n09x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n06x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nanb02aa1n02x5               g031(.a(new_n125), .b(new_n126), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  nor022aa1n16x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand42aa1n04x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanb02aa1n03x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n126), .b(new_n125), .c(new_n124), .d(new_n97), .o1(new_n132));
  nano22aa1n02x4               g037(.a(new_n129), .b(new_n126), .c(new_n130), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n127), .c(new_n124), .d(new_n97), .o1(new_n134));
  aobi12aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n131), .out0(\s[11] ));
  inv000aa1n04x5               g040(.a(new_n129), .o1(new_n136));
  xorc02aa1n02x5               g041(.a(\a[12] ), .b(\b[11] ), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n136), .out0(\s[12] ));
  oai012aa1n06x5               g043(.a(new_n98), .b(new_n99), .c(new_n103), .o1(new_n139));
  aoai13aa1n06x5               g044(.a(new_n115), .b(new_n119), .c(new_n118), .d(new_n116), .o1(new_n140));
  inv030aa1n02x5               g045(.a(new_n140), .o1(new_n141));
  oaib12aa1n18x5               g046(.a(new_n141), .b(new_n114), .c(new_n139), .out0(new_n142));
  nano23aa1n03x7               g047(.a(new_n131), .b(new_n127), .c(new_n137), .d(new_n122), .out0(new_n143));
  inv040aa1d32x5               g048(.a(\a[12] ), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\b[11] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  nand22aa1n03x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand23aa1n03x5               g052(.a(new_n146), .b(new_n130), .c(new_n147), .o1(new_n148));
  oai112aa1n06x5               g053(.a(new_n136), .b(new_n126), .c(new_n125), .d(new_n121), .o1(new_n149));
  tech160nm_fioaoi03aa1n03p5x5 g054(.a(new_n144), .b(new_n145), .c(new_n129), .o1(new_n150));
  oai012aa1n12x5               g055(.a(new_n150), .b(new_n149), .c(new_n148), .o1(new_n151));
  xorc02aa1n12x5               g056(.a(\a[13] ), .b(\b[12] ), .out0(new_n152));
  aoai13aa1n09x5               g057(.a(new_n152), .b(new_n151), .c(new_n142), .d(new_n143), .o1(new_n153));
  aoi112aa1n02x5               g058(.a(new_n152), .b(new_n151), .c(new_n142), .d(new_n143), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(\a[13] ), .o1(new_n156));
  inv000aa1d42x5               g061(.a(\b[12] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(new_n157), .b(new_n156), .o1(new_n158));
  xorc02aa1n12x5               g063(.a(\a[14] ), .b(\b[13] ), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n158), .out0(\s[14] ));
  inv000aa1d42x5               g065(.a(\a[14] ), .o1(new_n161));
  aboi22aa1d24x5               g066(.a(\b[13] ), .b(new_n161), .c(new_n156), .d(new_n157), .out0(new_n162));
  nand42aa1n04x5               g067(.a(new_n153), .b(new_n162), .o1(new_n163));
  nand42aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nor002aa1n20x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nano22aa1n02x4               g071(.a(new_n166), .b(new_n164), .c(new_n165), .out0(new_n167));
  nand22aa1n03x5               g072(.a(new_n163), .b(new_n167), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n166), .o1(new_n169));
  aoi022aa1n03x5               g074(.a(new_n163), .b(new_n164), .c(new_n169), .d(new_n165), .o1(new_n170));
  norb02aa1n02x7               g075(.a(new_n168), .b(new_n170), .out0(\s[15] ));
  inv000aa1n02x5               g076(.a(new_n167), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n169), .b(new_n172), .c(new_n153), .d(new_n162), .o1(new_n173));
  nor002aa1n16x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand02aa1d28x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  norp02aa1n02x5               g081(.a(new_n176), .b(new_n166), .o1(new_n177));
  aoi022aa1n03x5               g082(.a(new_n173), .b(new_n176), .c(new_n168), .d(new_n177), .o1(\s[16] ));
  inv000aa1n02x5               g083(.a(new_n113), .o1(new_n179));
  nano22aa1n03x5               g084(.a(new_n179), .b(new_n107), .c(new_n108), .out0(new_n180));
  nano22aa1n03x7               g085(.a(new_n131), .b(new_n146), .c(new_n147), .out0(new_n181));
  nano23aa1n03x7               g086(.a(new_n125), .b(new_n121), .c(new_n126), .d(new_n97), .out0(new_n182));
  nano23aa1d15x5               g087(.a(new_n174), .b(new_n166), .c(new_n175), .d(new_n165), .out0(new_n183));
  nand23aa1d12x5               g088(.a(new_n183), .b(new_n152), .c(new_n159), .o1(new_n184));
  nano22aa1d15x5               g089(.a(new_n184), .b(new_n182), .c(new_n181), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n140), .c(new_n180), .d(new_n139), .o1(new_n186));
  xroi22aa1d04x5               g091(.a(new_n156), .b(\b[12] ), .c(new_n161), .d(\b[13] ), .out0(new_n187));
  oai012aa1n02x5               g092(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .o1(new_n188));
  nanb03aa1n03x5               g093(.a(new_n174), .b(new_n175), .c(new_n165), .out0(new_n189));
  aoi012aa1n02x7               g094(.a(new_n174), .b(new_n166), .c(new_n175), .o1(new_n190));
  oai013aa1n03x4               g095(.a(new_n190), .b(new_n189), .c(new_n162), .d(new_n188), .o1(new_n191));
  aoi013aa1n06x4               g096(.a(new_n191), .b(new_n151), .c(new_n183), .d(new_n187), .o1(new_n192));
  tech160nm_finand02aa1n05x5   g097(.a(new_n186), .b(new_n192), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  aoi113aa1n02x5               g099(.a(new_n191), .b(new_n194), .c(new_n151), .d(new_n183), .e(new_n187), .o1(new_n195));
  aoi022aa1n02x5               g100(.a(new_n193), .b(new_n194), .c(new_n186), .d(new_n195), .o1(\s[17] ));
  nor002aa1d32x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  inv030aa1n04x5               g103(.a(new_n151), .o1(new_n199));
  oabi12aa1n12x5               g104(.a(new_n191), .b(new_n199), .c(new_n184), .out0(new_n200));
  aoai13aa1n06x5               g105(.a(new_n194), .b(new_n200), .c(new_n142), .d(new_n185), .o1(new_n201));
  nor042aa1n12x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand02aa1d08x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb02aa1n06x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n201), .c(new_n198), .out0(\s[18] ));
  nor002aa1d24x5               g110(.a(new_n202), .b(new_n197), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  nand42aa1n10x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  oai012aa1n18x5               g113(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .o1(new_n209));
  norb02aa1n03x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n207), .c(new_n193), .d(new_n194), .o1(new_n211));
  inv040aa1d32x5               g116(.a(\a[19] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[18] ), .o1(new_n213));
  nand02aa1d12x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(new_n214), .b(new_n208), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n203), .b(new_n207), .c(new_n193), .d(new_n194), .o1(new_n216));
  aobi12aa1n02x7               g121(.a(new_n211), .b(new_n216), .c(new_n215), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g123(.a(new_n210), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n214), .b(new_n219), .c(new_n201), .d(new_n206), .o1(new_n220));
  nor022aa1n16x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand42aa1n16x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n06x4               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[20] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[19] ), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n225), .b(new_n224), .o1(new_n226));
  aoi022aa1n02x5               g131(.a(new_n226), .b(new_n222), .c(new_n213), .d(new_n212), .o1(new_n227));
  aoi022aa1n03x5               g132(.a(new_n220), .b(new_n223), .c(new_n211), .d(new_n227), .o1(\s[20] ));
  nano32aa1n03x7               g133(.a(new_n215), .b(new_n194), .c(new_n223), .d(new_n204), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n200), .c(new_n142), .d(new_n185), .o1(new_n230));
  nanb03aa1d24x5               g135(.a(new_n221), .b(new_n222), .c(new_n208), .out0(new_n231));
  oaoi03aa1n09x5               g136(.a(\a[20] ), .b(\b[19] ), .c(new_n214), .o1(new_n232));
  inv000aa1n04x5               g137(.a(new_n232), .o1(new_n233));
  oai013aa1d12x5               g138(.a(new_n233), .b(new_n231), .c(new_n206), .d(new_n209), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nor002aa1d24x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nanp02aa1n04x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1d27x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n230), .c(new_n235), .out0(\s[21] ));
  aoai13aa1n03x5               g144(.a(new_n238), .b(new_n234), .c(new_n193), .d(new_n229), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n236), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n238), .o1(new_n242));
  aoai13aa1n02x5               g147(.a(new_n241), .b(new_n242), .c(new_n230), .d(new_n235), .o1(new_n243));
  nor042aa1n06x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nanp02aa1n04x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  aoib12aa1n02x5               g151(.a(new_n236), .b(new_n245), .c(new_n244), .out0(new_n247));
  aoi022aa1n03x5               g152(.a(new_n243), .b(new_n246), .c(new_n240), .d(new_n247), .o1(\s[22] ));
  inv040aa1n06x5               g153(.a(new_n229), .o1(new_n249));
  nona23aa1n06x5               g154(.a(new_n245), .b(new_n237), .c(new_n236), .d(new_n244), .out0(new_n250));
  nor042aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n200), .c(new_n142), .d(new_n185), .o1(new_n252));
  nano23aa1n09x5               g157(.a(new_n236), .b(new_n244), .c(new_n245), .d(new_n237), .out0(new_n253));
  oaih12aa1n06x5               g158(.a(new_n245), .b(new_n244), .c(new_n236), .o1(new_n254));
  inv030aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  aoi012aa1n09x5               g160(.a(new_n255), .b(new_n234), .c(new_n253), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[23] ), .b(\b[22] ), .out0(new_n257));
  xnbna2aa1n03x5               g162(.a(new_n257), .b(new_n252), .c(new_n256), .out0(\s[23] ));
  inv000aa1d42x5               g163(.a(new_n256), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n257), .b(new_n259), .c(new_n193), .d(new_n251), .o1(new_n260));
  orn002aa1n02x5               g165(.a(\a[23] ), .b(\b[22] ), .o(new_n261));
  and002aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o(new_n262));
  aoai13aa1n02x7               g167(.a(new_n261), .b(new_n262), .c(new_n252), .d(new_n256), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[24] ), .b(\b[23] ), .out0(new_n264));
  norb02aa1n02x5               g169(.a(new_n261), .b(new_n264), .out0(new_n265));
  aoi022aa1n03x5               g170(.a(new_n263), .b(new_n264), .c(new_n260), .d(new_n265), .o1(\s[24] ));
  nano22aa1n03x7               g171(.a(new_n250), .b(new_n257), .c(new_n264), .out0(new_n267));
  norb02aa1n03x5               g172(.a(new_n267), .b(new_n249), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n200), .c(new_n142), .d(new_n185), .o1(new_n269));
  and002aa1n02x5               g174(.a(\b[23] ), .b(\a[24] ), .o(new_n270));
  nano22aa1n02x5               g175(.a(new_n221), .b(new_n208), .c(new_n222), .out0(new_n271));
  oab012aa1n02x4               g176(.a(new_n209), .b(new_n197), .c(new_n202), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n253), .b(new_n232), .c(new_n272), .d(new_n271), .o1(new_n273));
  oa0022aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(\a[23] ), .d(\b[22] ), .o(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n262), .c(new_n273), .d(new_n254), .o1(new_n275));
  nanb02aa1n02x5               g180(.a(new_n270), .b(new_n275), .out0(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n269), .c(new_n276), .out0(\s[25] ));
  inv000aa1d42x5               g183(.a(new_n262), .o1(new_n279));
  aoai13aa1n09x5               g184(.a(new_n279), .b(new_n255), .c(new_n234), .d(new_n253), .o1(new_n280));
  aoi022aa1n02x7               g185(.a(new_n280), .b(new_n274), .c(\b[23] ), .d(\a[24] ), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n277), .b(new_n281), .c(new_n193), .d(new_n268), .o1(new_n282));
  nor042aa1n06x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv040aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n277), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n284), .b(new_n285), .c(new_n269), .d(new_n276), .o1(new_n286));
  xnrc02aa1n12x5               g191(.a(\b[25] ), .b(\a[26] ), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  norb02aa1n02x5               g193(.a(new_n287), .b(new_n283), .out0(new_n289));
  aoi022aa1n03x5               g194(.a(new_n286), .b(new_n288), .c(new_n282), .d(new_n289), .o1(\s[26] ));
  norb02aa1n02x5               g195(.a(new_n277), .b(new_n287), .out0(new_n291));
  nano22aa1n09x5               g196(.a(new_n249), .b(new_n267), .c(new_n291), .out0(new_n292));
  aoai13aa1n12x5               g197(.a(new_n292), .b(new_n200), .c(new_n142), .d(new_n185), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(\b[25] ), .b(\a[26] ), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(\b[24] ), .b(\a[25] ), .o1(new_n295));
  oai012aa1n02x5               g200(.a(new_n295), .b(\b[25] ), .c(\a[26] ), .o1(new_n296));
  nano23aa1n03x7               g201(.a(new_n296), .b(new_n270), .c(new_n284), .d(new_n294), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  tech160nm_fiaoi012aa1n04x5   g204(.a(new_n299), .b(new_n275), .c(new_n297), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n300), .b(new_n293), .o1(new_n301));
  xorc02aa1n12x5               g206(.a(\a[27] ), .b(\b[26] ), .out0(new_n302));
  aoi112aa1n02x5               g207(.a(new_n302), .b(new_n299), .c(new_n275), .d(new_n297), .o1(new_n303));
  aoi022aa1n02x5               g208(.a(new_n301), .b(new_n302), .c(new_n293), .d(new_n303), .o1(\s[27] ));
  aobi12aa1n06x5               g209(.a(new_n292), .b(new_n186), .c(new_n192), .out0(new_n305));
  inv000aa1n02x5               g210(.a(new_n297), .o1(new_n306));
  aoai13aa1n04x5               g211(.a(new_n298), .b(new_n306), .c(new_n280), .d(new_n274), .o1(new_n307));
  oai012aa1n03x5               g212(.a(new_n302), .b(new_n307), .c(new_n305), .o1(new_n308));
  norp02aa1n02x5               g213(.a(\b[26] ), .b(\a[27] ), .o1(new_n309));
  inv000aa1n03x5               g214(.a(new_n309), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n302), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n310), .b(new_n311), .c(new_n300), .d(new_n293), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[28] ), .b(\b[27] ), .out0(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n309), .o1(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n308), .d(new_n314), .o1(\s[28] ));
  and002aa1n02x5               g220(.a(new_n313), .b(new_n302), .o(new_n316));
  oaih12aa1n02x5               g221(.a(new_n316), .b(new_n307), .c(new_n305), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n300), .d(new_n293), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n319), .b(new_n321), .out0(new_n322));
  aoi022aa1n03x5               g227(.a(new_n320), .b(new_n321), .c(new_n317), .d(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g228(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g229(.a(new_n311), .b(new_n313), .c(new_n321), .out0(new_n325));
  oai012aa1n02x5               g230(.a(new_n325), .b(new_n307), .c(new_n305), .o1(new_n326));
  inv000aa1n02x5               g231(.a(new_n325), .o1(new_n327));
  oaoi03aa1n02x5               g232(.a(\a[29] ), .b(\b[28] ), .c(new_n319), .o1(new_n328));
  inv000aa1n03x5               g233(.a(new_n328), .o1(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n327), .c(new_n300), .d(new_n293), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .out0(new_n331));
  and002aa1n02x5               g236(.a(\b[28] ), .b(\a[29] ), .o(new_n332));
  oabi12aa1n02x5               g237(.a(new_n331), .b(\a[29] ), .c(\b[28] ), .out0(new_n333));
  oab012aa1n02x4               g238(.a(new_n333), .b(new_n319), .c(new_n332), .out0(new_n334));
  aoi022aa1n03x5               g239(.a(new_n330), .b(new_n331), .c(new_n326), .d(new_n334), .o1(\s[30] ));
  nano32aa1n02x4               g240(.a(new_n311), .b(new_n331), .c(new_n313), .d(new_n321), .out0(new_n336));
  oaih12aa1n02x5               g241(.a(new_n336), .b(new_n307), .c(new_n305), .o1(new_n337));
  xorc02aa1n02x5               g242(.a(\a[31] ), .b(\b[30] ), .out0(new_n338));
  oao003aa1n02x5               g243(.a(\a[30] ), .b(\b[29] ), .c(new_n329), .carry(new_n339));
  norb02aa1n02x5               g244(.a(new_n339), .b(new_n338), .out0(new_n340));
  inv000aa1n02x5               g245(.a(new_n336), .o1(new_n341));
  aoai13aa1n03x5               g246(.a(new_n339), .b(new_n341), .c(new_n300), .d(new_n293), .o1(new_n342));
  aoi022aa1n03x5               g247(.a(new_n342), .b(new_n338), .c(new_n337), .d(new_n340), .o1(\s[31] ));
  inv000aa1d42x5               g248(.a(\a[3] ), .o1(new_n344));
  xorb03aa1n02x5               g249(.a(new_n103), .b(\b[2] ), .c(new_n344), .out0(\s[3] ));
  norp02aa1n02x5               g250(.a(\b[2] ), .b(\a[3] ), .o1(new_n346));
  norp02aa1n02x5               g251(.a(new_n99), .b(new_n103), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[4] ), .b(\b[3] ), .out0(new_n348));
  norp03aa1n02x5               g253(.a(new_n347), .b(new_n348), .c(new_n346), .o1(new_n349));
  aboi22aa1n03x5               g254(.a(new_n347), .b(new_n98), .c(\b[3] ), .d(\a[4] ), .out0(new_n350));
  oaoi13aa1n02x5               g255(.a(new_n349), .b(new_n350), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g256(.a(new_n350), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g257(.a(new_n106), .b(new_n105), .o1(new_n353));
  inv000aa1d42x5               g258(.a(new_n109), .o1(new_n354));
  nanp03aa1n02x5               g259(.a(new_n350), .b(new_n354), .c(new_n110), .o1(new_n355));
  nona32aa1n02x4               g260(.a(new_n355), .b(new_n109), .c(new_n106), .d(new_n105), .out0(new_n356));
  aoai13aa1n02x5               g261(.a(new_n356), .b(new_n353), .c(new_n355), .d(new_n354), .o1(\s[6] ));
  aboi22aa1n03x5               g262(.a(new_n106), .b(new_n356), .c(new_n111), .d(new_n116), .out0(new_n358));
  norb03aa1n02x5               g263(.a(new_n111), .b(new_n106), .c(new_n112), .out0(new_n359));
  nanp02aa1n02x5               g264(.a(new_n356), .b(new_n359), .o1(new_n360));
  norb02aa1n02x5               g265(.a(new_n360), .b(new_n358), .out0(\s[7] ));
  nanp02aa1n02x5               g266(.a(new_n360), .b(new_n116), .o1(new_n362));
  norp02aa1n02x5               g267(.a(new_n108), .b(new_n112), .o1(new_n363));
  aoi022aa1n02x5               g268(.a(new_n362), .b(new_n108), .c(new_n360), .d(new_n363), .o1(\s[8] ));
  oaib12aa1n02x5               g269(.a(new_n124), .b(new_n122), .c(new_n142), .out0(\s[9] ));
endmodule


