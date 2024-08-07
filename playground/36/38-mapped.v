// Benchmark "adder" written by ABC on Thu Jul 18 06:45:16 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n349, new_n352, new_n354, new_n355,
    new_n356, new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor042aa1n06x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nor042aa1n06x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n09x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .d(new_n98), .out0(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nanb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  xnrc02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .out0(new_n106));
  nor043aa1n03x5               g011(.a(new_n102), .b(new_n105), .c(new_n106), .o1(new_n107));
  nor002aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n03x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nanp02aa1n03x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  aoi012aa1n12x5               g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  nand22aa1n03x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\b[3] ), .o1(new_n113));
  nanb02aa1n02x5               g018(.a(\a[4] ), .b(new_n113), .out0(new_n114));
  nand42aa1n02x5               g019(.a(new_n114), .b(new_n112), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\a[3] ), .o1(new_n116));
  inv000aa1d42x5               g021(.a(\b[2] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n116), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n118), .b(new_n119), .o1(new_n120));
  norp02aa1n02x5               g025(.a(\b[3] ), .b(\a[4] ), .o1(new_n121));
  norp02aa1n02x5               g026(.a(\b[2] ), .b(\a[3] ), .o1(new_n122));
  aoi012aa1n02x7               g027(.a(new_n121), .b(new_n122), .c(new_n112), .o1(new_n123));
  oai013aa1d12x5               g028(.a(new_n123), .b(new_n111), .c(new_n115), .d(new_n120), .o1(new_n124));
  inv000aa1d42x5               g029(.a(new_n98), .o1(new_n125));
  nanp02aa1n02x5               g030(.a(new_n101), .b(new_n99), .o1(new_n126));
  norp02aa1n02x5               g031(.a(\b[4] ), .b(\a[5] ), .o1(new_n127));
  oai012aa1n02x5               g032(.a(new_n103), .b(new_n127), .c(new_n104), .o1(new_n128));
  oai112aa1n02x5               g033(.a(new_n125), .b(new_n126), .c(new_n102), .d(new_n128), .o1(new_n129));
  xorc02aa1n02x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n129), .c(new_n124), .d(new_n107), .o1(new_n131));
  xorc02aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n131), .c(new_n97), .out0(\s[10] ));
  oai022aa1d24x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  aoi022aa1n02x5               g040(.a(new_n131), .b(new_n135), .c(\b[9] ), .d(\a[10] ), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n06x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nor022aa1n08x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n138), .c(new_n136), .d(new_n139), .o1(new_n143));
  aoi112aa1n02x5               g048(.a(new_n142), .b(new_n138), .c(new_n136), .d(new_n139), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(\s[12] ));
  nona23aa1n02x4               g050(.a(new_n141), .b(new_n139), .c(new_n138), .d(new_n140), .out0(new_n146));
  nano22aa1n02x4               g051(.a(new_n146), .b(new_n130), .c(new_n132), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n129), .c(new_n124), .d(new_n107), .o1(new_n148));
  nor022aa1n04x5               g053(.a(new_n140), .b(new_n138), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n150));
  nanp02aa1n03x5               g055(.a(new_n134), .b(new_n150), .o1(new_n151));
  aoi022aa1d18x5               g056(.a(new_n151), .b(new_n149), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nor002aa1n16x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(new_n156));
  xobna2aa1n03x5               g061(.a(new_n156), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n154), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n148), .d(new_n153), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nona23aa1n02x4               g067(.a(new_n162), .b(new_n155), .c(new_n154), .d(new_n161), .out0(new_n163));
  oaoi03aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(new_n148), .d(new_n153), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n12x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nor043aa1n03x5               g074(.a(new_n111), .b(new_n115), .c(new_n120), .o1(new_n170));
  inv000aa1n02x5               g075(.a(new_n123), .o1(new_n171));
  oai012aa1n04x7               g076(.a(new_n107), .b(new_n170), .c(new_n171), .o1(new_n172));
  norp02aa1n02x5               g077(.a(new_n102), .b(new_n128), .o1(new_n173));
  nano22aa1n02x5               g078(.a(new_n173), .b(new_n125), .c(new_n126), .out0(new_n174));
  nano23aa1n02x4               g079(.a(new_n138), .b(new_n140), .c(new_n141), .d(new_n139), .out0(new_n175));
  nanp03aa1n02x5               g080(.a(new_n175), .b(new_n130), .c(new_n132), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n153), .b(new_n176), .c(new_n172), .d(new_n174), .o1(new_n177));
  nano23aa1n02x4               g082(.a(new_n154), .b(new_n161), .c(new_n162), .d(new_n155), .out0(new_n178));
  nand42aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nanb02aa1n09x5               g084(.a(new_n168), .b(new_n179), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n181), .b(new_n164), .c(new_n177), .d(new_n178), .o1(new_n182));
  nor002aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand22aa1n04x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n12x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  tech160nm_fiaoi012aa1n02p5x5 g090(.a(new_n185), .b(new_n182), .c(new_n169), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n185), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n168), .b(new_n187), .c(new_n166), .d(new_n179), .o1(new_n188));
  norp02aa1n02x5               g093(.a(new_n186), .b(new_n188), .o1(\s[16] ));
  nona22aa1n03x5               g094(.a(new_n178), .b(new_n180), .c(new_n185), .out0(new_n190));
  nor042aa1n02x5               g095(.a(new_n176), .b(new_n190), .o1(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n129), .c(new_n124), .d(new_n107), .o1(new_n192));
  nor043aa1n03x5               g097(.a(new_n163), .b(new_n180), .c(new_n185), .o1(new_n193));
  oai112aa1n02x5               g098(.a(new_n162), .b(new_n179), .c(new_n161), .d(new_n154), .o1(new_n194));
  nona22aa1n02x4               g099(.a(new_n194), .b(new_n183), .c(new_n168), .out0(new_n195));
  aoi022aa1d18x5               g100(.a(new_n193), .b(new_n152), .c(new_n184), .d(new_n195), .o1(new_n196));
  xnrc02aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .out0(new_n197));
  xobna2aa1n03x5               g102(.a(new_n197), .b(new_n192), .c(new_n196), .out0(\s[17] ));
  nor022aa1n16x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n192), .d(new_n196), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  inv020aa1n04x5               g108(.a(\a[18] ), .o1(new_n204));
  xroi22aa1d06x4               g109(.a(new_n203), .b(\b[16] ), .c(new_n204), .d(\b[17] ), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  oaoi03aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n206), .c(new_n192), .d(new_n196), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(new_n147), .b(new_n193), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n196), .b(new_n214), .c(new_n172), .d(new_n174), .o1(new_n215));
  nand22aa1n03x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nanb02aa1n12x5               g121(.a(new_n212), .b(new_n216), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n207), .c(new_n215), .d(new_n205), .o1(new_n219));
  xnrc02aa1n12x5               g124(.a(\b[19] ), .b(\a[20] ), .out0(new_n220));
  aoi012aa1n03x5               g125(.a(new_n220), .b(new_n219), .c(new_n213), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n220), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n212), .b(new_n222), .c(new_n209), .d(new_n216), .o1(new_n223));
  nor002aa1n02x5               g128(.a(new_n221), .b(new_n223), .o1(\s[20] ));
  nona22aa1n12x5               g129(.a(new_n205), .b(new_n217), .c(new_n220), .out0(new_n225));
  norp02aa1n02x5               g130(.a(\b[17] ), .b(\a[18] ), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(\b[17] ), .b(\a[18] ), .o1(new_n227));
  oai112aa1n06x5               g132(.a(new_n227), .b(new_n216), .c(new_n226), .d(new_n199), .o1(new_n228));
  oab012aa1n12x5               g133(.a(new_n212), .b(\a[20] ), .c(\b[19] ), .out0(new_n229));
  aoi022aa1n12x5               g134(.a(new_n228), .b(new_n229), .c(\b[19] ), .d(\a[20] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  aoai13aa1n04x5               g136(.a(new_n231), .b(new_n225), .c(new_n192), .d(new_n196), .o1(new_n232));
  xorb03aa1n02x5               g137(.a(new_n232), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  inv000aa1n06x5               g139(.a(new_n234), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n225), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n234), .out0(new_n238));
  aoai13aa1n03x5               g143(.a(new_n238), .b(new_n230), .c(new_n215), .d(new_n236), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\a[22] ), .o1(new_n240));
  inv000aa1d42x5               g145(.a(\b[21] ), .o1(new_n241));
  nand22aa1n03x5               g146(.a(new_n241), .b(new_n240), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nand02aa1d06x5               g148(.a(new_n242), .b(new_n243), .o1(new_n244));
  aoi012aa1n03x5               g149(.a(new_n244), .b(new_n239), .c(new_n235), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n244), .o1(new_n246));
  aoi112aa1n03x4               g151(.a(new_n234), .b(new_n246), .c(new_n232), .d(new_n238), .o1(new_n247));
  norp02aa1n02x5               g152(.a(new_n245), .b(new_n247), .o1(\s[22] ));
  nano22aa1n03x7               g153(.a(new_n244), .b(new_n235), .c(new_n237), .out0(new_n249));
  nona23aa1n08x5               g154(.a(new_n205), .b(new_n249), .c(new_n220), .d(new_n217), .out0(new_n250));
  nand42aa1n02x5               g155(.a(new_n228), .b(new_n229), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[19] ), .b(\a[20] ), .o1(new_n252));
  nano32aa1n03x7               g157(.a(new_n244), .b(new_n235), .c(new_n237), .d(new_n252), .out0(new_n253));
  oaoi03aa1n02x5               g158(.a(\a[22] ), .b(\b[21] ), .c(new_n235), .o1(new_n254));
  aoi012aa1n09x5               g159(.a(new_n254), .b(new_n251), .c(new_n253), .o1(new_n255));
  aoai13aa1n04x5               g160(.a(new_n255), .b(new_n250), .c(new_n192), .d(new_n196), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  inv040aa1n03x5               g164(.a(new_n250), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n255), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n215), .d(new_n260), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[24] ), .b(\b[23] ), .out0(new_n264));
  aobi12aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n259), .out0(new_n265));
  aoi112aa1n03x4               g170(.a(new_n258), .b(new_n264), .c(new_n256), .d(new_n262), .o1(new_n266));
  nor002aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(\s[24] ));
  nano32aa1d12x5               g172(.a(new_n225), .b(new_n264), .c(new_n249), .d(new_n262), .out0(new_n268));
  inv020aa1n03x5               g173(.a(new_n268), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n226), .b(new_n199), .o1(new_n270));
  nano22aa1n02x4               g175(.a(new_n270), .b(new_n227), .c(new_n216), .out0(new_n271));
  inv000aa1n02x5               g176(.a(new_n229), .o1(new_n272));
  tech160nm_fioai012aa1n05x5   g177(.a(new_n253), .b(new_n271), .c(new_n272), .o1(new_n273));
  inv000aa1n02x5               g178(.a(new_n254), .o1(new_n274));
  nand22aa1n03x5               g179(.a(new_n264), .b(new_n262), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n276));
  aoai13aa1n12x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .d(new_n274), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n278), .b(new_n269), .c(new_n192), .d(new_n196), .o1(new_n279));
  xorb03aa1n02x5               g184(.a(new_n279), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n277), .c(new_n215), .d(new_n268), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  aobi12aa1n06x5               g190(.a(new_n285), .b(new_n284), .c(new_n282), .out0(new_n286));
  aoi112aa1n03x4               g191(.a(new_n281), .b(new_n285), .c(new_n279), .d(new_n283), .o1(new_n287));
  nor002aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(\s[26] ));
  nano23aa1n02x4               g193(.a(new_n101), .b(new_n98), .c(new_n99), .d(new_n100), .out0(new_n289));
  nona22aa1n02x4               g194(.a(new_n289), .b(new_n106), .c(new_n105), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n111), .o1(new_n291));
  nano23aa1n02x4               g196(.a(new_n122), .b(new_n121), .c(new_n119), .d(new_n112), .out0(new_n292));
  aoi012aa1n02x5               g197(.a(new_n171), .b(new_n292), .c(new_n291), .o1(new_n293));
  oai012aa1n02x5               g198(.a(new_n174), .b(new_n293), .c(new_n290), .o1(new_n294));
  nanp02aa1n02x5               g199(.a(new_n195), .b(new_n184), .o1(new_n295));
  oaib12aa1n02x5               g200(.a(new_n295), .b(new_n190), .c(new_n152), .out0(new_n296));
  inv000aa1n02x5               g201(.a(new_n275), .o1(new_n297));
  and002aa1n02x5               g202(.a(new_n285), .b(new_n283), .o(new_n298));
  nano22aa1d15x5               g203(.a(new_n250), .b(new_n297), .c(new_n298), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n296), .c(new_n294), .d(new_n191), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .carry(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  tech160nm_fiaoi012aa1n05x5   g207(.a(new_n302), .b(new_n277), .c(new_n298), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  xnbna2aa1n03x5               g209(.a(new_n304), .b(new_n300), .c(new_n303), .out0(\s[27] ));
  nor042aa1n03x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  inv040aa1n03x5               g211(.a(new_n306), .o1(new_n307));
  aoai13aa1n02x5               g212(.a(new_n297), .b(new_n254), .c(new_n251), .d(new_n253), .o1(new_n308));
  inv000aa1n02x5               g213(.a(new_n298), .o1(new_n309));
  aoai13aa1n06x5               g214(.a(new_n301), .b(new_n309), .c(new_n308), .d(new_n276), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n304), .b(new_n310), .c(new_n215), .d(new_n299), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[27] ), .b(\a[28] ), .out0(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n312), .b(new_n311), .c(new_n307), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n304), .o1(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n314), .b(new_n300), .c(new_n303), .o1(new_n315));
  nano22aa1n03x5               g220(.a(new_n315), .b(new_n307), .c(new_n312), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n313), .b(new_n316), .o1(\s[28] ));
  norb02aa1n02x5               g222(.a(new_n304), .b(new_n312), .out0(new_n318));
  aoai13aa1n06x5               g223(.a(new_n318), .b(new_n310), .c(new_n215), .d(new_n299), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[28] ), .b(\b[27] ), .c(new_n307), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[28] ), .b(\a[29] ), .out0(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n321), .b(new_n319), .c(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n318), .o1(new_n323));
  tech160nm_fiaoi012aa1n02p5x5 g228(.a(new_n323), .b(new_n300), .c(new_n303), .o1(new_n324));
  nano22aa1n03x5               g229(.a(new_n324), .b(new_n320), .c(new_n321), .out0(new_n325));
  norp02aa1n03x5               g230(.a(new_n322), .b(new_n325), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g232(.a(new_n304), .b(new_n321), .c(new_n312), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n310), .c(new_n215), .d(new_n299), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[29] ), .b(\b[28] ), .c(new_n320), .carry(new_n330));
  xnrc02aa1n02x5               g235(.a(\b[29] ), .b(\a[30] ), .out0(new_n331));
  tech160nm_fiaoi012aa1n02p5x5 g236(.a(new_n331), .b(new_n329), .c(new_n330), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n328), .o1(new_n333));
  tech160nm_fiaoi012aa1n03p5x5 g238(.a(new_n333), .b(new_n300), .c(new_n303), .o1(new_n334));
  nano22aa1n03x5               g239(.a(new_n334), .b(new_n330), .c(new_n331), .out0(new_n335));
  norp02aa1n03x5               g240(.a(new_n332), .b(new_n335), .o1(\s[30] ));
  norb02aa1n02x5               g241(.a(new_n328), .b(new_n331), .out0(new_n337));
  aoai13aa1n06x5               g242(.a(new_n337), .b(new_n310), .c(new_n215), .d(new_n299), .o1(new_n338));
  oaoi03aa1n02x5               g243(.a(\a[30] ), .b(\b[29] ), .c(new_n330), .o1(new_n339));
  nanb02aa1n02x5               g244(.a(\b[30] ), .b(\a[31] ), .out0(new_n340));
  nanb02aa1n02x5               g245(.a(\a[31] ), .b(\b[30] ), .out0(new_n341));
  nanp02aa1n02x5               g246(.a(new_n341), .b(new_n340), .o1(new_n342));
  nona22aa1n02x5               g247(.a(new_n338), .b(new_n339), .c(new_n342), .out0(new_n343));
  inv000aa1n02x5               g248(.a(new_n339), .o1(new_n344));
  aoi022aa1n02x7               g249(.a(new_n338), .b(new_n344), .c(new_n341), .d(new_n340), .o1(new_n345));
  norb02aa1n03x4               g250(.a(new_n343), .b(new_n345), .out0(\s[31] ));
  xnbna2aa1n03x5               g251(.a(new_n111), .b(new_n119), .c(new_n118), .out0(\s[3] ));
  orn002aa1n02x5               g252(.a(new_n111), .b(new_n120), .o(new_n348));
  aoi022aa1n02x5               g253(.a(new_n114), .b(new_n112), .c(new_n116), .d(new_n117), .o1(new_n349));
  aoi022aa1n02x5               g254(.a(new_n124), .b(new_n114), .c(new_n349), .d(new_n348), .o1(\s[4] ));
  xorb03aa1n02x5               g255(.a(new_n124), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g256(.a(\a[5] ), .b(\b[4] ), .c(new_n293), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g258(.a(new_n101), .o1(new_n354));
  norp02aa1n02x5               g259(.a(new_n106), .b(new_n105), .o1(new_n355));
  aobi12aa1n02x5               g260(.a(new_n128), .b(new_n124), .c(new_n355), .out0(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n356), .b(new_n100), .c(new_n354), .out0(\s[7] ));
  oaoi03aa1n02x5               g262(.a(\a[7] ), .b(\b[6] ), .c(new_n356), .o1(new_n358));
  xorb03aa1n02x5               g263(.a(new_n358), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g264(.a(new_n130), .b(new_n172), .c(new_n174), .out0(\s[9] ));
endmodule


