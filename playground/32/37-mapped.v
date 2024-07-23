// Benchmark "adder" written by ABC on Thu Jul 18 04:41:39 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n346, new_n348, new_n349, new_n350, new_n352, new_n353,
    new_n354, new_n356, new_n357, new_n358, new_n360, new_n361, new_n362,
    new_n363, new_n364, new_n366, new_n367, new_n369, new_n370;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand42aa1d28x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d20x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aoi012aa1d24x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[2] ), .b(\a[3] ), .out0(new_n103));
  oa0022aa1n06x5               g008(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n104));
  oai012aa1n18x5               g009(.a(new_n104), .b(new_n103), .c(new_n102), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  inv040aa1n08x5               g011(.a(new_n106), .o1(new_n107));
  nand42aa1n10x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  oai112aa1n06x5               g013(.a(new_n107), .b(new_n108), .c(\b[7] ), .d(\a[8] ), .o1(new_n109));
  nor042aa1d18x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  tech160nm_fiaoi012aa1n03p5x5 g015(.a(new_n110), .b(\a[8] ), .c(\b[7] ), .o1(new_n111));
  aoi022aa1d24x5               g016(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n112));
  nor042aa1n09x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  aoi012aa1n12x5               g018(.a(new_n113), .b(\a[6] ), .c(\b[5] ), .o1(new_n114));
  nano32aa1n03x7               g019(.a(new_n109), .b(new_n114), .c(new_n111), .d(new_n112), .out0(new_n115));
  aoi022aa1n12x5               g020(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n116));
  oai012aa1n06x5               g021(.a(new_n116), .b(new_n113), .c(new_n110), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nand02aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  aoi012aa1n06x5               g024(.a(new_n118), .b(new_n106), .c(new_n119), .o1(new_n120));
  oai012aa1n06x5               g025(.a(new_n120), .b(new_n109), .c(new_n117), .o1(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n115), .d(new_n105), .o1(new_n123));
  xorc02aa1n02x5               g028(.a(\a[10] ), .b(\b[9] ), .out0(new_n124));
  nand42aa1d28x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  oai022aa1d24x5               g030(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nanp03aa1n02x5               g032(.a(new_n123), .b(new_n125), .c(new_n127), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n124), .c(new_n98), .d(new_n123), .o1(\s[10] ));
  nanp02aa1n02x5               g034(.a(new_n126), .b(new_n125), .o1(new_n130));
  nand42aa1n16x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norp02aa1n09x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n09x5               g037(.a(new_n97), .b(new_n132), .c(new_n125), .d(new_n131), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n121), .c(new_n115), .d(new_n105), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand22aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1d21x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n130), .out0(\s[11] ));
  inv030aa1n03x5               g043(.a(new_n135), .o1(new_n139));
  aob012aa1n02x5               g044(.a(new_n137), .b(new_n134), .c(new_n130), .out0(new_n140));
  nor042aa1n09x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand22aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n06x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  nona23aa1n02x4               g048(.a(new_n140), .b(new_n142), .c(new_n141), .d(new_n135), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n139), .d(new_n140), .o1(\s[12] ));
  and003aa1n06x5               g050(.a(new_n133), .b(new_n143), .c(new_n137), .o(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n121), .c(new_n115), .d(new_n105), .o1(new_n147));
  nano22aa1n03x7               g052(.a(new_n141), .b(new_n136), .c(new_n142), .out0(new_n148));
  nano22aa1n03x7               g053(.a(new_n127), .b(new_n139), .c(new_n125), .out0(new_n149));
  aoi012aa1n12x5               g054(.a(new_n141), .b(new_n135), .c(new_n142), .o1(new_n150));
  inv000aa1n02x5               g055(.a(new_n150), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n148), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n147), .b(new_n152), .o1(new_n153));
  xorc02aa1n12x5               g058(.a(\a[13] ), .b(\b[12] ), .out0(new_n154));
  aoi112aa1n02x5               g059(.a(new_n151), .b(new_n154), .c(new_n149), .d(new_n148), .o1(new_n155));
  aoi022aa1n02x5               g060(.a(new_n153), .b(new_n154), .c(new_n147), .d(new_n155), .o1(\s[13] ));
  norp02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  inv000aa1n03x5               g062(.a(new_n157), .o1(new_n158));
  nanp02aa1n06x5               g063(.a(new_n115), .b(new_n105), .o1(new_n159));
  inv000aa1n06x5               g064(.a(new_n121), .o1(new_n160));
  nand02aa1n04x5               g065(.a(new_n159), .b(new_n160), .o1(new_n161));
  oai112aa1n04x5               g066(.a(new_n139), .b(new_n125), .c(new_n132), .d(new_n97), .o1(new_n162));
  oaib12aa1n09x5               g067(.a(new_n150), .b(new_n162), .c(new_n148), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n154), .b(new_n163), .c(new_n161), .d(new_n146), .o1(new_n164));
  xorc02aa1n03x5               g069(.a(\a[14] ), .b(\b[13] ), .out0(new_n165));
  oai022aa1d18x5               g070(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n166));
  aoi012aa1n02x5               g071(.a(new_n166), .b(\a[14] ), .c(\b[13] ), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(new_n164), .b(new_n167), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n165), .c(new_n158), .d(new_n164), .o1(\s[14] ));
  nanp02aa1n02x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  xnrc02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .out0(new_n171));
  nano22aa1n03x7               g076(.a(new_n171), .b(new_n158), .c(new_n170), .out0(new_n172));
  oaoi03aa1n02x5               g077(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n173));
  xorc02aa1n12x5               g078(.a(\a[15] ), .b(\b[14] ), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n173), .c(new_n153), .d(new_n172), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n174), .b(new_n173), .c(new_n153), .d(new_n172), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n175), .b(new_n176), .out0(\s[15] ));
  inv000aa1d42x5               g082(.a(\a[15] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[14] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  nor002aa1n16x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  and002aa1n12x5               g086(.a(\b[15] ), .b(\a[16] ), .o(new_n182));
  nor042aa1n06x5               g087(.a(new_n182), .b(new_n181), .o1(new_n183));
  aoi112aa1n02x5               g088(.a(new_n182), .b(new_n181), .c(new_n178), .d(new_n179), .o1(new_n184));
  nanp02aa1n02x5               g089(.a(new_n175), .b(new_n184), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n183), .c(new_n180), .d(new_n175), .o1(\s[16] ));
  nanp03aa1n02x5               g091(.a(new_n172), .b(new_n174), .c(new_n183), .o1(new_n187));
  nano32aa1n02x5               g092(.a(new_n187), .b(new_n133), .c(new_n137), .d(new_n143), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n161), .b(new_n188), .o1(new_n189));
  tech160nm_finand02aa1n03p5x5 g094(.a(new_n174), .b(new_n183), .o1(new_n190));
  nano22aa1n06x5               g095(.a(new_n190), .b(new_n154), .c(new_n165), .out0(new_n191));
  nand02aa1d04x5               g096(.a(new_n146), .b(new_n191), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n181), .o1(new_n193));
  aoi022aa1d24x5               g098(.a(\b[14] ), .b(\a[15] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(new_n166), .b(new_n194), .o1(new_n195));
  aoai13aa1n03x5               g100(.a(new_n193), .b(new_n182), .c(new_n195), .d(new_n180), .o1(new_n196));
  aoi012aa1n06x5               g101(.a(new_n196), .b(new_n163), .c(new_n191), .o1(new_n197));
  aoai13aa1n12x5               g102(.a(new_n197), .b(new_n192), .c(new_n159), .d(new_n160), .o1(new_n198));
  xorc02aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .out0(new_n199));
  aoi022aa1n02x5               g104(.a(new_n195), .b(new_n180), .c(\a[16] ), .d(\b[15] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n193), .out0(new_n201));
  aoi112aa1n02x5               g106(.a(new_n201), .b(new_n200), .c(new_n163), .d(new_n191), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n198), .b(new_n199), .c(new_n189), .d(new_n202), .o1(\s[17] ));
  nor042aa1d18x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n204), .o1(new_n205));
  inv000aa1d42x5               g110(.a(\a[17] ), .o1(new_n206));
  oaib12aa1n06x5               g111(.a(new_n198), .b(new_n206), .c(\b[16] ), .out0(new_n207));
  nor042aa1n04x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand42aa1n08x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n205), .out0(\s[18] ));
  nand42aa1n03x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  nano23aa1n06x5               g117(.a(new_n204), .b(new_n208), .c(new_n209), .d(new_n212), .out0(new_n213));
  oaoi03aa1n02x5               g118(.a(\a[18] ), .b(\b[17] ), .c(new_n205), .o1(new_n214));
  nor042aa1n09x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nanp02aa1n04x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n214), .c(new_n198), .d(new_n213), .o1(new_n218));
  aoi112aa1n02x7               g123(.a(new_n217), .b(new_n214), .c(new_n198), .d(new_n213), .o1(new_n219));
  norb02aa1n03x4               g124(.a(new_n218), .b(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv030aa1n02x5               g126(.a(new_n215), .o1(new_n222));
  nor042aa1n04x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand02aa1n06x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  norb03aa1n02x5               g130(.a(new_n224), .b(new_n215), .c(new_n223), .out0(new_n226));
  nanp02aa1n03x5               g131(.a(new_n218), .b(new_n226), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n225), .c(new_n218), .d(new_n222), .o1(\s[20] ));
  nano23aa1n06x5               g133(.a(new_n215), .b(new_n223), .c(new_n224), .d(new_n216), .out0(new_n229));
  nand02aa1d06x5               g134(.a(new_n229), .b(new_n213), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nanb03aa1n06x5               g136(.a(new_n223), .b(new_n224), .c(new_n216), .out0(new_n232));
  oai112aa1n03x5               g137(.a(new_n222), .b(new_n209), .c(new_n208), .d(new_n204), .o1(new_n233));
  tech160nm_fiaoi012aa1n05x5   g138(.a(new_n223), .b(new_n215), .c(new_n224), .o1(new_n234));
  tech160nm_fioai012aa1n05x5   g139(.a(new_n234), .b(new_n233), .c(new_n232), .o1(new_n235));
  nor002aa1d32x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n235), .c(new_n198), .d(new_n231), .o1(new_n239));
  nano22aa1n03x7               g144(.a(new_n223), .b(new_n216), .c(new_n224), .out0(new_n240));
  tech160nm_fioai012aa1n03p5x5 g145(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .o1(new_n241));
  oab012aa1n06x5               g146(.a(new_n241), .b(new_n204), .c(new_n208), .out0(new_n242));
  inv040aa1n03x5               g147(.a(new_n234), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n243), .b(new_n238), .c(new_n242), .d(new_n240), .o1(new_n244));
  aobi12aa1n02x7               g149(.a(new_n244), .b(new_n198), .c(new_n231), .out0(new_n245));
  norb02aa1n03x4               g150(.a(new_n239), .b(new_n245), .out0(\s[21] ));
  inv000aa1d42x5               g151(.a(new_n236), .o1(new_n247));
  nor042aa1n02x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  nand42aa1n02x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  norb03aa1n02x5               g155(.a(new_n249), .b(new_n236), .c(new_n248), .out0(new_n251));
  nanp02aa1n03x5               g156(.a(new_n239), .b(new_n251), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n250), .c(new_n239), .d(new_n247), .o1(\s[22] ));
  nano23aa1n06x5               g158(.a(new_n236), .b(new_n248), .c(new_n249), .d(new_n237), .out0(new_n254));
  and003aa1n02x5               g159(.a(new_n229), .b(new_n213), .c(new_n254), .o(new_n255));
  aoai13aa1n06x5               g160(.a(new_n254), .b(new_n243), .c(new_n242), .d(new_n240), .o1(new_n256));
  oaoi03aa1n12x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n256), .b(new_n258), .o1(new_n259));
  tech160nm_fixorc02aa1n05x5   g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n198), .d(new_n255), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n257), .c(new_n235), .d(new_n254), .o1(new_n262));
  aobi12aa1n02x7               g167(.a(new_n262), .b(new_n198), .c(new_n255), .out0(new_n263));
  norb02aa1n03x4               g168(.a(new_n261), .b(new_n263), .out0(\s[23] ));
  nor042aa1n04x5               g169(.a(\b[22] ), .b(\a[23] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  nor002aa1n02x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  and002aa1n03x5               g172(.a(\b[23] ), .b(\a[24] ), .o(new_n268));
  nor002aa1n03x5               g173(.a(new_n268), .b(new_n267), .o1(new_n269));
  norp03aa1n02x5               g174(.a(new_n268), .b(new_n267), .c(new_n265), .o1(new_n270));
  nanp02aa1n03x5               g175(.a(new_n261), .b(new_n270), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n261), .d(new_n266), .o1(\s[24] ));
  oabi12aa1n03x5               g177(.a(new_n196), .b(new_n187), .c(new_n152), .out0(new_n273));
  nano32aa1n03x7               g178(.a(new_n230), .b(new_n269), .c(new_n254), .d(new_n260), .out0(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n273), .c(new_n161), .d(new_n188), .o1(new_n275));
  and002aa1n03x5               g180(.a(new_n260), .b(new_n269), .o(new_n276));
  inv020aa1n06x5               g181(.a(new_n276), .o1(new_n277));
  oab012aa1n02x4               g182(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n277), .c(new_n256), .d(new_n258), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n198), .d(new_n274), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n276), .b(new_n257), .c(new_n235), .d(new_n254), .o1(new_n282));
  nano22aa1n02x4               g187(.a(new_n280), .b(new_n282), .c(new_n278), .out0(new_n283));
  aobi12aa1n02x7               g188(.a(new_n281), .b(new_n283), .c(new_n275), .out0(\s[25] ));
  norp02aa1n02x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  tech160nm_fixorc02aa1n05x5   g191(.a(\a[26] ), .b(\b[25] ), .out0(new_n287));
  nanp02aa1n02x5               g192(.a(\b[25] ), .b(\a[26] ), .o1(new_n288));
  oai022aa1n02x5               g193(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n289));
  norb02aa1n02x5               g194(.a(new_n288), .b(new_n289), .out0(new_n290));
  nanp02aa1n03x5               g195(.a(new_n281), .b(new_n290), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n287), .c(new_n281), .d(new_n286), .o1(\s[26] ));
  and002aa1n09x5               g197(.a(new_n287), .b(new_n280), .o(new_n293));
  nano23aa1n06x5               g198(.a(new_n277), .b(new_n230), .c(new_n293), .d(new_n254), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n273), .c(new_n161), .d(new_n188), .o1(new_n295));
  aoi022aa1n06x5               g200(.a(new_n279), .b(new_n293), .c(new_n288), .d(new_n289), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n296), .b(new_n295), .o1(new_n297));
  nor042aa1n04x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  and002aa1n18x5               g203(.a(\b[26] ), .b(\a[27] ), .o(new_n299));
  norp02aa1n06x5               g204(.a(new_n299), .b(new_n298), .o1(new_n300));
  aoi122aa1n02x5               g205(.a(new_n300), .b(new_n288), .c(new_n289), .d(new_n279), .e(new_n293), .o1(new_n301));
  aoi022aa1n02x5               g206(.a(new_n297), .b(new_n300), .c(new_n301), .d(new_n295), .o1(\s[27] ));
  inv000aa1d42x5               g207(.a(new_n293), .o1(new_n303));
  nanp02aa1n02x5               g208(.a(new_n289), .b(new_n288), .o1(new_n304));
  aoai13aa1n04x5               g209(.a(new_n304), .b(new_n303), .c(new_n282), .d(new_n278), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n299), .o1(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n305), .c(new_n198), .d(new_n294), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n298), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n299), .c(new_n296), .d(new_n295), .o1(new_n309));
  xorc02aa1n02x5               g214(.a(\a[28] ), .b(\b[27] ), .out0(new_n310));
  norp02aa1n02x5               g215(.a(new_n310), .b(new_n298), .o1(new_n311));
  aoi022aa1n03x5               g216(.a(new_n309), .b(new_n310), .c(new_n307), .d(new_n311), .o1(\s[28] ));
  inv000aa1n02x5               g217(.a(new_n300), .o1(new_n313));
  norb02aa1n02x5               g218(.a(new_n310), .b(new_n313), .out0(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n305), .c(new_n198), .d(new_n294), .o1(new_n315));
  inv000aa1n02x5               g220(.a(new_n314), .o1(new_n316));
  oao003aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n296), .d(new_n295), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n317), .b(new_n319), .out0(new_n320));
  aoi022aa1n03x5               g225(.a(new_n318), .b(new_n319), .c(new_n315), .d(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g227(.a(new_n310), .b(new_n319), .c(new_n300), .o(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n305), .c(new_n198), .d(new_n294), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n323), .o1(new_n325));
  inv000aa1d42x5               g230(.a(\b[28] ), .o1(new_n326));
  inv000aa1d42x5               g231(.a(\a[29] ), .o1(new_n327));
  oaib12aa1n02x5               g232(.a(new_n317), .b(\b[28] ), .c(new_n327), .out0(new_n328));
  oaib12aa1n02x5               g233(.a(new_n328), .b(new_n326), .c(\a[29] ), .out0(new_n329));
  aoai13aa1n03x5               g234(.a(new_n329), .b(new_n325), .c(new_n296), .d(new_n295), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .out0(new_n331));
  oaoi13aa1n02x5               g236(.a(new_n331), .b(new_n328), .c(new_n327), .d(new_n326), .o1(new_n332));
  aoi022aa1n03x5               g237(.a(new_n330), .b(new_n331), .c(new_n324), .d(new_n332), .o1(\s[30] ));
  nano32aa1n02x5               g238(.a(new_n313), .b(new_n331), .c(new_n310), .d(new_n319), .out0(new_n334));
  aoai13aa1n02x7               g239(.a(new_n334), .b(new_n305), .c(new_n198), .d(new_n294), .o1(new_n335));
  aoi022aa1n02x5               g240(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n336));
  norb02aa1n02x5               g241(.a(\a[31] ), .b(\b[30] ), .out0(new_n337));
  obai22aa1n02x7               g242(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n338));
  aoi112aa1n02x5               g243(.a(new_n338), .b(new_n337), .c(new_n328), .d(new_n336), .o1(new_n339));
  xorc02aa1n02x5               g244(.a(\a[31] ), .b(\b[30] ), .out0(new_n340));
  inv000aa1n02x5               g245(.a(new_n334), .o1(new_n341));
  norp02aa1n02x5               g246(.a(\b[29] ), .b(\a[30] ), .o1(new_n342));
  aoi012aa1n02x5               g247(.a(new_n342), .b(new_n328), .c(new_n336), .o1(new_n343));
  aoai13aa1n03x5               g248(.a(new_n343), .b(new_n341), .c(new_n296), .d(new_n295), .o1(new_n344));
  aoi022aa1n03x5               g249(.a(new_n344), .b(new_n340), .c(new_n335), .d(new_n339), .o1(\s[31] ));
  inv000aa1d42x5               g250(.a(\a[3] ), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n102), .b(\b[2] ), .c(new_n346), .out0(\s[3] ));
  norp02aa1n02x5               g252(.a(new_n103), .b(new_n102), .o1(new_n348));
  xorc02aa1n02x5               g253(.a(\a[4] ), .b(\b[3] ), .out0(new_n349));
  aoib12aa1n02x5               g254(.a(new_n349), .b(new_n346), .c(\b[2] ), .out0(new_n350));
  aboi22aa1n03x5               g255(.a(new_n348), .b(new_n350), .c(new_n105), .d(new_n349), .out0(\s[4] ));
  nanp02aa1n02x5               g256(.a(\b[3] ), .b(\a[4] ), .o1(new_n352));
  nanp02aa1n02x5               g257(.a(\b[4] ), .b(\a[5] ), .o1(new_n353));
  nanb02aa1n02x5               g258(.a(new_n113), .b(new_n353), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n105), .c(new_n352), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g260(.a(new_n354), .b(new_n105), .c(new_n352), .o(new_n356));
  nanp02aa1n02x5               g261(.a(\b[5] ), .b(\a[6] ), .o1(new_n357));
  norb02aa1n02x5               g262(.a(new_n357), .b(new_n110), .out0(new_n358));
  xobna2aa1n03x5               g263(.a(new_n358), .b(new_n356), .c(new_n353), .out0(\s[6] ));
  norb02aa1n02x5               g264(.a(new_n108), .b(new_n106), .out0(new_n360));
  aoai13aa1n02x5               g265(.a(new_n353), .b(new_n113), .c(new_n105), .d(new_n352), .o1(new_n361));
  oaoi03aa1n02x5               g266(.a(\a[6] ), .b(\b[5] ), .c(new_n361), .o1(new_n362));
  nano22aa1n02x4               g267(.a(new_n106), .b(new_n108), .c(new_n357), .out0(new_n363));
  aobi12aa1n02x5               g268(.a(new_n363), .b(new_n361), .c(new_n358), .out0(new_n364));
  oab012aa1n02x4               g269(.a(new_n364), .b(new_n362), .c(new_n360), .out0(\s[7] ));
  inv000aa1n03x5               g270(.a(new_n364), .o1(new_n366));
  norb02aa1n02x5               g271(.a(new_n119), .b(new_n118), .out0(new_n367));
  xnbna2aa1n03x5               g272(.a(new_n367), .b(new_n366), .c(new_n107), .out0(\s[8] ));
  oaib12aa1n02x5               g273(.a(new_n120), .b(new_n97), .c(new_n131), .out0(new_n369));
  oab012aa1n02x4               g274(.a(new_n369), .b(new_n109), .c(new_n117), .out0(new_n370));
  aoi022aa1n02x5               g275(.a(new_n161), .b(new_n122), .c(new_n159), .d(new_n370), .o1(\s[9] ));
endmodule


