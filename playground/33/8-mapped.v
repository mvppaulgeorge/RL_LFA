// Benchmark "adder" written by ABC on Thu Jul 18 04:54:33 2024

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
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n346, new_n347, new_n348, new_n350, new_n351, new_n353,
    new_n354, new_n356, new_n357, new_n358, new_n360, new_n361, new_n363,
    new_n365;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1d28x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  oai122aa1n12x5               g002(.a(new_n97), .b(\a[4] ), .c(\b[3] ), .d(\a[3] ), .e(\b[2] ), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1n08x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n06x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  nand22aa1n09x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand02aa1d12x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand22aa1n06x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor042aa1n12x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  tech160nm_fioai012aa1n03p5x5 g011(.a(new_n103), .b(new_n106), .c(new_n105), .o1(new_n107));
  oai013aa1n09x5               g012(.a(new_n107), .b(new_n98), .c(new_n101), .d(new_n104), .o1(new_n108));
  nand42aa1n08x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nand42aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nona23aa1n02x5               g017(.a(new_n109), .b(new_n112), .c(new_n111), .d(new_n110), .out0(new_n113));
  norp02aa1n09x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norb02aa1n06x4               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nor002aa1d32x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand22aa1n12x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  norb02aa1n09x5               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nano22aa1n03x7               g024(.a(new_n113), .b(new_n116), .c(new_n119), .out0(new_n120));
  nand22aa1n09x5               g025(.a(new_n120), .b(new_n108), .o1(new_n121));
  nanb03aa1n03x5               g026(.a(new_n114), .b(new_n115), .c(new_n109), .out0(new_n122));
  inv000aa1n06x5               g027(.a(new_n117), .o1(new_n123));
  oai112aa1n03x5               g028(.a(new_n123), .b(new_n118), .c(new_n111), .d(new_n110), .o1(new_n124));
  norp02aa1n02x5               g029(.a(new_n124), .b(new_n122), .o1(new_n125));
  nor042aa1d18x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  oaoi03aa1n03x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n127));
  nona32aa1n02x4               g032(.a(new_n121), .b(new_n127), .c(new_n126), .d(new_n125), .out0(new_n128));
  nor002aa1d32x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n16x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(new_n131));
  nand22aa1n12x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n131), .b(new_n128), .c(new_n132), .out0(\s[10] ));
  nand02aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nano22aa1n02x4               g040(.a(new_n135), .b(new_n130), .c(new_n134), .out0(new_n136));
  aoai13aa1n06x5               g041(.a(new_n136), .b(new_n129), .c(new_n128), .d(new_n132), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n135), .b(new_n134), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n130), .b(new_n129), .c(new_n128), .d(new_n132), .o1(new_n139));
  aobi12aa1n02x5               g044(.a(new_n137), .b(new_n139), .c(new_n138), .out0(\s[11] ));
  inv030aa1n03x5               g045(.a(new_n135), .o1(new_n141));
  nor002aa1d32x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n06x4               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  nona23aa1n02x4               g049(.a(new_n137), .b(new_n143), .c(new_n142), .d(new_n135), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n144), .c(new_n137), .d(new_n141), .o1(\s[12] ));
  inv020aa1n02x5               g051(.a(new_n98), .o1(new_n147));
  norp02aa1n03x5               g052(.a(new_n101), .b(new_n104), .o1(new_n148));
  oai022aa1n02x5               g053(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n149));
  aoi022aa1n03x5               g054(.a(new_n148), .b(new_n147), .c(new_n103), .d(new_n149), .o1(new_n150));
  nano23aa1n03x5               g055(.a(new_n111), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n151));
  nano23aa1n02x4               g056(.a(new_n114), .b(new_n117), .c(new_n118), .d(new_n115), .out0(new_n152));
  nand02aa1n02x5               g057(.a(new_n152), .b(new_n151), .o1(new_n153));
  oab012aa1n06x5               g058(.a(new_n127), .b(new_n124), .c(new_n122), .out0(new_n154));
  tech160nm_fioai012aa1n03p5x5 g059(.a(new_n154), .b(new_n150), .c(new_n153), .o1(new_n155));
  nona23aa1n03x5               g060(.a(new_n134), .b(new_n130), .c(new_n135), .d(new_n129), .out0(new_n156));
  norb02aa1n06x5               g061(.a(new_n132), .b(new_n126), .out0(new_n157));
  nano22aa1n03x7               g062(.a(new_n156), .b(new_n144), .c(new_n157), .out0(new_n158));
  nanp02aa1n03x5               g063(.a(new_n155), .b(new_n158), .o1(new_n159));
  nanb03aa1n03x5               g064(.a(new_n142), .b(new_n143), .c(new_n134), .out0(new_n160));
  oai112aa1n06x5               g065(.a(new_n141), .b(new_n130), .c(new_n129), .d(new_n126), .o1(new_n161));
  tech160nm_fioai012aa1n03p5x5 g066(.a(new_n143), .b(new_n142), .c(new_n135), .o1(new_n162));
  tech160nm_fioai012aa1n05x5   g067(.a(new_n162), .b(new_n161), .c(new_n160), .o1(new_n163));
  nanb02aa1n03x5               g068(.a(new_n163), .b(new_n159), .out0(new_n164));
  nor042aa1n06x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand02aa1d24x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n06x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoi012aa1n02x5               g072(.a(new_n135), .b(\a[10] ), .c(\b[9] ), .o1(new_n168));
  nano22aa1n02x4               g073(.a(new_n142), .b(new_n134), .c(new_n143), .out0(new_n169));
  oai112aa1n02x5               g074(.a(new_n169), .b(new_n168), .c(new_n129), .d(new_n126), .o1(new_n170));
  nano22aa1n02x4               g075(.a(new_n167), .b(new_n170), .c(new_n162), .out0(new_n171));
  aoi022aa1n02x5               g076(.a(new_n164), .b(new_n167), .c(new_n159), .d(new_n171), .o1(\s[13] ));
  aoi012aa1n02x5               g077(.a(new_n165), .b(new_n164), .c(new_n166), .o1(new_n173));
  nor042aa1n06x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand22aa1n09x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n06x4               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  oai022aa1d18x5               g081(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n177));
  aoi122aa1n06x5               g082(.a(new_n177), .b(\b[13] ), .c(\a[14] ), .d(new_n164), .e(new_n167), .o1(new_n178));
  oabi12aa1n03x5               g083(.a(new_n178), .b(new_n173), .c(new_n176), .out0(\s[14] ));
  nano23aa1n03x5               g084(.a(new_n165), .b(new_n174), .c(new_n175), .d(new_n166), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n163), .c(new_n155), .d(new_n158), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n174), .b(new_n165), .c(new_n175), .o1(new_n182));
  nor002aa1d32x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  nand02aa1d28x5               g088(.a(\b[14] ), .b(\a[15] ), .o1(new_n184));
  nanb02aa1n06x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  xobna2aa1n03x5               g090(.a(new_n185), .b(new_n181), .c(new_n182), .out0(\s[15] ));
  inv000aa1d42x5               g091(.a(new_n183), .o1(new_n187));
  aoai13aa1n03x5               g092(.a(new_n187), .b(new_n185), .c(new_n181), .d(new_n182), .o1(new_n188));
  nor042aa1d18x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand02aa1d10x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  nanb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(new_n191));
  norb03aa1n02x5               g096(.a(new_n190), .b(new_n183), .c(new_n189), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n185), .c(new_n181), .d(new_n182), .o1(new_n193));
  aob012aa1n03x5               g098(.a(new_n193), .b(new_n188), .c(new_n191), .out0(\s[16] ));
  nona22aa1n03x5               g099(.a(new_n180), .b(new_n185), .c(new_n191), .out0(new_n195));
  norb02aa1n02x7               g100(.a(new_n158), .b(new_n195), .out0(new_n196));
  nanp02aa1n02x5               g101(.a(new_n155), .b(new_n196), .o1(new_n197));
  nona23aa1n09x5               g102(.a(new_n190), .b(new_n184), .c(new_n183), .d(new_n189), .out0(new_n198));
  nano22aa1n12x5               g103(.a(new_n198), .b(new_n167), .c(new_n176), .out0(new_n199));
  nand22aa1n03x5               g104(.a(new_n158), .b(new_n199), .o1(new_n200));
  nanb03aa1n03x5               g105(.a(new_n189), .b(new_n190), .c(new_n184), .out0(new_n201));
  oai112aa1n04x5               g106(.a(new_n177), .b(new_n175), .c(\b[14] ), .d(\a[15] ), .o1(new_n202));
  tech160nm_fioai012aa1n03p5x5 g107(.a(new_n190), .b(new_n189), .c(new_n183), .o1(new_n203));
  oai012aa1n12x5               g108(.a(new_n203), .b(new_n202), .c(new_n201), .o1(new_n204));
  aoi012aa1n12x5               g109(.a(new_n204), .b(new_n163), .c(new_n199), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n205), .b(new_n200), .c(new_n121), .d(new_n154), .o1(new_n206));
  tech160nm_fixorc02aa1n05x5   g111(.a(\a[17] ), .b(\b[16] ), .out0(new_n207));
  inv000aa1n02x5               g112(.a(new_n207), .o1(new_n208));
  oai112aa1n02x5               g113(.a(new_n208), .b(new_n203), .c(new_n202), .d(new_n201), .o1(new_n209));
  aoi012aa1n02x5               g114(.a(new_n209), .b(new_n163), .c(new_n199), .o1(new_n210));
  aoi022aa1n02x5               g115(.a(new_n206), .b(new_n207), .c(new_n197), .d(new_n210), .o1(\s[17] ));
  nor022aa1n16x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  aoi012aa1n02x5               g117(.a(new_n212), .b(new_n206), .c(new_n207), .o1(new_n213));
  nor002aa1d32x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand42aa1n10x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  norb02aa1n03x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  norb03aa1n02x5               g121(.a(new_n215), .b(new_n212), .c(new_n214), .out0(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n208), .c(new_n197), .d(new_n205), .o1(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n213), .c(new_n216), .o1(\s[18] ));
  and002aa1n02x5               g124(.a(new_n207), .b(new_n216), .o(new_n220));
  norp02aa1n03x5               g125(.a(new_n214), .b(new_n212), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n215), .b(new_n221), .out0(new_n222));
  nor002aa1d24x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nand42aa1n06x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  norb02aa1n06x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n222), .c(new_n206), .d(new_n220), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(new_n225), .b(new_n222), .c(new_n206), .d(new_n220), .o1(new_n227));
  norb02aa1n03x4               g132(.a(new_n226), .b(new_n227), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g134(.a(new_n223), .o1(new_n230));
  nor042aa1n04x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand42aa1n20x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  norb02aa1n03x5               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  nona23aa1n03x5               g138(.a(new_n226), .b(new_n232), .c(new_n231), .d(new_n223), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n233), .c(new_n230), .d(new_n226), .o1(\s[20] ));
  nano32aa1n03x7               g140(.a(new_n208), .b(new_n233), .c(new_n216), .d(new_n225), .out0(new_n236));
  nano22aa1n03x7               g141(.a(new_n231), .b(new_n224), .c(new_n232), .out0(new_n237));
  tech160nm_fioai012aa1n04x5   g142(.a(new_n215), .b(\b[18] ), .c(\a[19] ), .o1(new_n238));
  nona22aa1n09x5               g143(.a(new_n237), .b(new_n221), .c(new_n238), .out0(new_n239));
  oaih12aa1n02x5               g144(.a(new_n232), .b(new_n231), .c(new_n223), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n239), .b(new_n240), .o1(new_n241));
  xorc02aa1n12x5               g146(.a(\a[21] ), .b(\b[20] ), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n241), .c(new_n206), .d(new_n236), .o1(new_n243));
  nano22aa1n02x4               g148(.a(new_n242), .b(new_n239), .c(new_n240), .out0(new_n244));
  aobi12aa1n02x5               g149(.a(new_n244), .b(new_n206), .c(new_n236), .out0(new_n245));
  norb02aa1n03x4               g150(.a(new_n243), .b(new_n245), .out0(\s[21] ));
  nor042aa1n03x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  inv000aa1n03x5               g152(.a(new_n247), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[22] ), .b(\b[21] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(\a[22] ), .o1(new_n250));
  inv000aa1d42x5               g155(.a(\b[21] ), .o1(new_n251));
  aoi012aa1n02x5               g156(.a(new_n247), .b(new_n250), .c(new_n251), .o1(new_n252));
  oai112aa1n03x5               g157(.a(new_n243), .b(new_n252), .c(new_n251), .d(new_n250), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n248), .d(new_n243), .o1(\s[22] ));
  nand02aa1d08x5               g159(.a(new_n249), .b(new_n242), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n236), .b(new_n255), .out0(new_n256));
  tech160nm_fioaoi03aa1n03p5x5 g161(.a(\a[22] ), .b(\b[21] ), .c(new_n248), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n255), .c(new_n239), .d(new_n240), .o1(new_n259));
  tech160nm_fixorc02aa1n03p5x5 g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n206), .d(new_n256), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n255), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n257), .c(new_n241), .d(new_n262), .o1(new_n263));
  aobi12aa1n02x5               g168(.a(new_n263), .b(new_n206), .c(new_n256), .out0(new_n264));
  norb02aa1n03x4               g169(.a(new_n261), .b(new_n264), .out0(\s[23] ));
  nor042aa1n03x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  tech160nm_fixorc02aa1n03p5x5 g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  and002aa1n02x5               g173(.a(\b[23] ), .b(\a[24] ), .o(new_n269));
  oai022aa1n02x5               g174(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n270));
  nona22aa1n03x5               g175(.a(new_n261), .b(new_n269), .c(new_n270), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n268), .c(new_n267), .d(new_n261), .o1(\s[24] ));
  and002aa1n02x5               g177(.a(new_n268), .b(new_n260), .o(new_n273));
  and003aa1n02x5               g178(.a(new_n236), .b(new_n273), .c(new_n262), .o(new_n274));
  nanp02aa1n02x5               g179(.a(new_n259), .b(new_n273), .o1(new_n275));
  oaib12aa1n02x5               g180(.a(new_n275), .b(new_n269), .c(new_n270), .out0(new_n276));
  tech160nm_fixorc02aa1n03p5x5 g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n206), .d(new_n274), .o1(new_n278));
  oaoi03aa1n02x5               g183(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n275), .b(new_n277), .c(new_n279), .out0(new_n280));
  aoi012aa1n02x5               g185(.a(new_n280), .b(new_n206), .c(new_n274), .o1(new_n281));
  norb02aa1n03x4               g186(.a(new_n278), .b(new_n281), .out0(\s[25] ));
  nor042aa1n06x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  and002aa1n02x5               g190(.a(\b[25] ), .b(\a[26] ), .o(new_n286));
  oai022aa1n02x5               g191(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n287));
  nona22aa1n03x5               g192(.a(new_n278), .b(new_n286), .c(new_n287), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n284), .d(new_n278), .o1(\s[26] ));
  and002aa1n02x5               g194(.a(new_n285), .b(new_n277), .o(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n279), .c(new_n259), .d(new_n273), .o1(new_n291));
  inv000aa1n02x5               g196(.a(new_n204), .o1(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n195), .c(new_n170), .d(new_n162), .o1(new_n293));
  nand42aa1n02x5               g198(.a(new_n262), .b(new_n273), .o1(new_n294));
  nano22aa1n03x7               g199(.a(new_n294), .b(new_n236), .c(new_n290), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n155), .d(new_n196), .o1(new_n296));
  oaoi03aa1n02x5               g201(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n297), .o1(new_n298));
  nand23aa1n03x5               g203(.a(new_n296), .b(new_n291), .c(new_n298), .o1(new_n299));
  xorc02aa1n12x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  aoi112aa1n03x4               g205(.a(new_n300), .b(new_n297), .c(new_n206), .d(new_n295), .o1(new_n301));
  aoi022aa1n02x7               g206(.a(new_n299), .b(new_n300), .c(new_n301), .d(new_n291), .o1(\s[27] ));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n303), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n299), .b(new_n300), .o1(new_n305));
  tech160nm_fixorc02aa1n03p5x5 g210(.a(\a[28] ), .b(\b[27] ), .out0(new_n306));
  tech160nm_fiaoi012aa1n05x5   g211(.a(new_n297), .b(new_n206), .c(new_n295), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n300), .o1(new_n308));
  oai022aa1n02x5               g213(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n309), .b(\a[28] ), .c(\b[27] ), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n308), .c(new_n307), .d(new_n291), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n306), .c(new_n305), .d(new_n304), .o1(\s[28] ));
  and002aa1n02x5               g217(.a(new_n306), .b(new_n300), .o(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  aob012aa1n09x5               g219(.a(new_n309), .b(\b[27] ), .c(\a[28] ), .out0(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n315), .b(new_n316), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n314), .c(new_n307), .d(new_n291), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n315), .o1(new_n319));
  aoai13aa1n03x5               g224(.a(new_n316), .b(new_n319), .c(new_n299), .d(new_n313), .o1(new_n320));
  nanp02aa1n03x5               g225(.a(new_n320), .b(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g227(.a(new_n316), .b(new_n300), .c(new_n306), .out0(new_n323));
  oaoi03aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .o1(new_n324));
  norp02aa1n02x5               g229(.a(\b[29] ), .b(\a[30] ), .o1(new_n325));
  nanp02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .o1(new_n326));
  nanb02aa1n02x5               g231(.a(new_n325), .b(new_n326), .out0(new_n327));
  aoai13aa1n02x7               g232(.a(new_n327), .b(new_n324), .c(new_n299), .d(new_n323), .o1(new_n328));
  inv000aa1d42x5               g233(.a(new_n323), .o1(new_n329));
  nanp02aa1n02x5               g234(.a(\b[28] ), .b(\a[29] ), .o1(new_n330));
  oai012aa1n02x5               g235(.a(new_n315), .b(\b[28] ), .c(\a[29] ), .o1(new_n331));
  aoi012aa1n02x5               g236(.a(new_n327), .b(new_n331), .c(new_n330), .o1(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n329), .c(new_n307), .d(new_n291), .o1(new_n333));
  nanp02aa1n03x5               g238(.a(new_n328), .b(new_n333), .o1(\s[30] ));
  nano23aa1n02x4               g239(.a(new_n327), .b(new_n316), .c(new_n306), .d(new_n300), .out0(new_n335));
  nanp02aa1n02x5               g240(.a(new_n299), .b(new_n335), .o1(new_n336));
  inv000aa1n02x5               g241(.a(new_n335), .o1(new_n337));
  nano22aa1n02x4               g242(.a(new_n325), .b(new_n330), .c(new_n326), .out0(new_n338));
  aoi012aa1n02x5               g243(.a(new_n325), .b(\a[31] ), .c(\b[30] ), .o1(new_n339));
  oai012aa1n02x5               g244(.a(new_n339), .b(\b[30] ), .c(\a[31] ), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n340), .b(new_n331), .c(new_n338), .o1(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n337), .c(new_n307), .d(new_n291), .o1(new_n342));
  aoi012aa1n02x5               g247(.a(new_n325), .b(new_n331), .c(new_n338), .o1(new_n343));
  xorc02aa1n02x5               g248(.a(\a[31] ), .b(\b[30] ), .out0(new_n344));
  aoai13aa1n03x5               g249(.a(new_n342), .b(new_n344), .c(new_n336), .d(new_n343), .o1(\s[31] ));
  norb02aa1n02x5               g250(.a(new_n102), .b(new_n106), .out0(new_n346));
  oai112aa1n02x5               g251(.a(new_n346), .b(new_n97), .c(new_n100), .d(new_n99), .o1(new_n347));
  oaoi13aa1n02x5               g252(.a(new_n346), .b(new_n97), .c(new_n99), .d(new_n100), .o1(new_n348));
  norb02aa1n02x5               g253(.a(new_n347), .b(new_n348), .out0(\s[3] ));
  inv000aa1d42x5               g254(.a(new_n106), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n103), .b(new_n105), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n347), .c(new_n350), .out0(\s[4] ));
  norb02aa1n02x5               g257(.a(new_n112), .b(new_n111), .out0(new_n353));
  aoi122aa1n02x5               g258(.a(new_n353), .b(new_n149), .c(new_n103), .d(new_n148), .e(new_n147), .o1(new_n354));
  aoi012aa1n02x5               g259(.a(new_n354), .b(new_n108), .c(new_n353), .o1(\s[5] ));
  nanb02aa1n02x5               g260(.a(new_n110), .b(new_n109), .out0(new_n356));
  aoai13aa1n02x5               g261(.a(new_n356), .b(new_n111), .c(new_n108), .d(new_n112), .o1(new_n357));
  nona22aa1n02x4               g262(.a(new_n109), .b(new_n110), .c(new_n111), .out0(new_n358));
  aoai13aa1n02x5               g263(.a(new_n357), .b(new_n358), .c(new_n353), .d(new_n108), .o1(\s[6] ));
  oai012aa1n02x5               g264(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n360));
  nanp02aa1n02x5               g265(.a(new_n108), .b(new_n151), .o1(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n119), .b(new_n361), .c(new_n360), .out0(\s[7] ));
  aob012aa1n02x5               g267(.a(new_n119), .b(new_n361), .c(new_n360), .out0(new_n363));
  xnbna2aa1n03x5               g268(.a(new_n116), .b(new_n363), .c(new_n123), .out0(\s[8] ));
  norp03aa1n02x5               g269(.a(new_n125), .b(new_n127), .c(new_n157), .o1(new_n365));
  aoi022aa1n02x5               g270(.a(new_n155), .b(new_n157), .c(new_n121), .d(new_n365), .o1(\s[9] ));
endmodule


