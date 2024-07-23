// Benchmark "adder" written by ABC on Thu Jul 18 04:56:24 2024

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
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n302, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n321, new_n323, new_n324,
    new_n327, new_n329, new_n330, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1d28x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand42aa1n20x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nor042aa1d18x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nano22aa1n03x7               g004(.a(new_n99), .b(new_n97), .c(new_n98), .out0(new_n100));
  oai112aa1n06x5               g005(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n101));
  nand02aa1d24x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norp02aa1n12x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n06x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nand23aa1n04x5               g009(.a(new_n100), .b(new_n101), .c(new_n104), .o1(new_n105));
  tech160nm_fioai012aa1n03p5x5 g010(.a(new_n102), .b(new_n103), .c(new_n99), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  nand22aa1n12x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor042aa1n03x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n108), .b(new_n109), .out0(new_n110));
  nor042aa1d18x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nand02aa1d04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  norb02aa1n03x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  norp02aa1n24x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand42aa1d28x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norb02aa1n02x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nor002aa1d32x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nand42aa1d28x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanb02aa1n02x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nano32aa1n03x7               g024(.a(new_n119), .b(new_n116), .c(new_n113), .d(new_n110), .out0(new_n120));
  nano23aa1n09x5               g025(.a(new_n114), .b(new_n117), .c(new_n118), .d(new_n115), .out0(new_n121));
  inv040aa1n03x5               g026(.a(new_n111), .o1(new_n122));
  oaoi03aa1n12x5               g027(.a(\a[6] ), .b(\b[5] ), .c(new_n122), .o1(new_n123));
  nona22aa1n09x5               g028(.a(new_n115), .b(new_n117), .c(new_n114), .out0(new_n124));
  aoi022aa1n09x5               g029(.a(new_n121), .b(new_n123), .c(new_n115), .d(new_n124), .o1(new_n125));
  inv000aa1n02x5               g030(.a(new_n125), .o1(new_n126));
  aoi012aa1n06x5               g031(.a(new_n126), .b(new_n107), .c(new_n120), .o1(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(\a[9] ), .b(\b[8] ), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n06x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nor042aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n16x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nano22aa1n02x4               g038(.a(new_n132), .b(new_n131), .c(new_n133), .out0(new_n134));
  oai012aa1n03x5               g039(.a(new_n134), .b(new_n128), .c(new_n130), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n132), .b(new_n133), .out0(new_n136));
  oai012aa1n02x5               g041(.a(new_n131), .b(new_n128), .c(new_n130), .o1(new_n137));
  aobi12aa1n02x7               g042(.a(new_n135), .b(new_n137), .c(new_n136), .out0(\s[11] ));
  aoi022aa1d24x5               g043(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n139));
  oaoi13aa1n03x5               g044(.a(new_n132), .b(new_n139), .c(new_n128), .d(new_n130), .o1(new_n140));
  xorc02aa1n12x5               g045(.a(\a[12] ), .b(\b[11] ), .out0(new_n141));
  nand42aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  oaih22aa1n06x5               g047(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n143));
  nanb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  obai22aa1n03x5               g049(.a(new_n135), .b(new_n144), .c(new_n140), .d(new_n141), .out0(\s[12] ));
  nano23aa1n06x5               g050(.a(new_n111), .b(new_n109), .c(new_n112), .d(new_n108), .out0(new_n146));
  nand02aa1d04x5               g051(.a(new_n121), .b(new_n146), .o1(new_n147));
  aoai13aa1n12x5               g052(.a(new_n125), .b(new_n147), .c(new_n105), .d(new_n106), .o1(new_n148));
  nano23aa1d15x5               g053(.a(new_n130), .b(new_n132), .c(new_n133), .d(new_n131), .out0(new_n149));
  xnrc02aa1n06x5               g054(.a(\b[8] ), .b(\a[9] ), .out0(new_n150));
  nanb03aa1d24x5               g055(.a(new_n150), .b(new_n149), .c(new_n141), .out0(new_n151));
  inv000aa1n02x5               g056(.a(new_n151), .o1(new_n152));
  nanp02aa1n06x5               g057(.a(new_n148), .b(new_n152), .o1(new_n153));
  oai022aa1d24x5               g058(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n154));
  aoai13aa1n12x5               g059(.a(new_n142), .b(new_n143), .c(new_n154), .d(new_n139), .o1(new_n155));
  xorc02aa1n12x5               g060(.a(\a[13] ), .b(\b[12] ), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n153), .c(new_n155), .out0(\s[13] ));
  inv040aa1d32x5               g062(.a(\a[13] ), .o1(new_n158));
  inv040aa1n18x5               g063(.a(\b[12] ), .o1(new_n159));
  nanp02aa1n04x5               g064(.a(new_n159), .b(new_n158), .o1(new_n160));
  nand02aa1d04x5               g065(.a(new_n153), .b(new_n155), .o1(new_n161));
  nanp02aa1n06x5               g066(.a(new_n161), .b(new_n156), .o1(new_n162));
  xorc02aa1n12x5               g067(.a(\a[14] ), .b(\b[13] ), .out0(new_n163));
  and002aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o(new_n164));
  oai022aa1n02x5               g069(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n165));
  nona22aa1n03x5               g070(.a(new_n162), .b(new_n164), .c(new_n165), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n163), .c(new_n160), .d(new_n162), .o1(\s[14] ));
  inv000aa1d42x5               g072(.a(\a[14] ), .o1(new_n168));
  xroi22aa1d04x5               g073(.a(new_n158), .b(\b[12] ), .c(new_n168), .d(\b[13] ), .out0(new_n169));
  oaoi03aa1n12x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n170));
  xorc02aa1n12x5               g075(.a(\a[15] ), .b(\b[14] ), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n170), .c(new_n161), .d(new_n169), .o1(new_n172));
  aoi112aa1n02x7               g077(.a(new_n171), .b(new_n170), .c(new_n161), .d(new_n169), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  orn002aa1n03x5               g079(.a(\a[15] ), .b(\b[14] ), .o(new_n175));
  tech160nm_fixorc02aa1n02p5x5 g080(.a(\a[16] ), .b(\b[15] ), .out0(new_n176));
  nand02aa1d16x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  oaih22aa1d12x5               g083(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n179));
  nona22aa1n03x5               g084(.a(new_n172), .b(new_n178), .c(new_n179), .out0(new_n180));
  aoai13aa1n03x5               g085(.a(new_n180), .b(new_n176), .c(new_n175), .d(new_n172), .o1(\s[16] ));
  nand42aa1n03x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nor042aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nano32aa1n03x7               g088(.a(new_n183), .b(new_n175), .c(new_n177), .d(new_n182), .out0(new_n184));
  nano32aa1d12x5               g089(.a(new_n151), .b(new_n184), .c(new_n156), .d(new_n163), .out0(new_n185));
  nanp02aa1n06x5               g090(.a(new_n148), .b(new_n185), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n163), .b(new_n156), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n176), .b(new_n171), .o1(new_n188));
  aoi022aa1n03x5               g093(.a(new_n184), .b(new_n170), .c(new_n177), .d(new_n179), .o1(new_n189));
  oai013aa1d12x5               g094(.a(new_n189), .b(new_n187), .c(new_n155), .d(new_n188), .o1(new_n190));
  nanb02aa1n06x5               g095(.a(new_n190), .b(new_n186), .out0(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  inv040aa1n08x5               g098(.a(new_n193), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  aoai13aa1n04x5               g100(.a(new_n195), .b(new_n190), .c(new_n148), .d(new_n185), .o1(new_n196));
  xorc02aa1n12x5               g101(.a(\a[18] ), .b(\b[17] ), .out0(new_n197));
  inv000aa1d42x5               g102(.a(\a[18] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[17] ), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n193), .b(new_n198), .c(new_n199), .o1(new_n200));
  oai112aa1n03x5               g105(.a(new_n196), .b(new_n200), .c(new_n199), .d(new_n198), .o1(new_n201));
  aoai13aa1n02x5               g106(.a(new_n201), .b(new_n197), .c(new_n194), .d(new_n196), .o1(\s[18] ));
  and002aa1n02x5               g107(.a(new_n197), .b(new_n195), .o(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n190), .c(new_n148), .d(new_n185), .o1(new_n204));
  oaoi03aa1n12x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n194), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  nor002aa1n16x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand42aa1d28x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g116(.a(new_n207), .o1(new_n212));
  aob012aa1n03x5               g117(.a(new_n209), .b(new_n204), .c(new_n206), .out0(new_n213));
  nor002aa1n04x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand42aa1d28x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  norb03aa1n02x5               g121(.a(new_n215), .b(new_n207), .c(new_n214), .out0(new_n217));
  nand42aa1n02x5               g122(.a(new_n213), .b(new_n217), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n216), .c(new_n212), .d(new_n213), .o1(\s[20] ));
  nano23aa1n09x5               g124(.a(new_n207), .b(new_n214), .c(new_n215), .d(new_n208), .out0(new_n220));
  nand23aa1n06x5               g125(.a(new_n220), .b(new_n195), .c(new_n197), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  oaoi03aa1n02x5               g127(.a(\a[20] ), .b(\b[19] ), .c(new_n212), .o1(new_n223));
  tech160nm_fiao0012aa1n02p5x5 g128(.a(new_n223), .b(new_n220), .c(new_n205), .o(new_n224));
  xorc02aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .out0(new_n225));
  aoai13aa1n04x5               g130(.a(new_n225), .b(new_n224), .c(new_n191), .d(new_n222), .o1(new_n226));
  aoi112aa1n02x7               g131(.a(new_n225), .b(new_n224), .c(new_n191), .d(new_n222), .o1(new_n227));
  norb02aa1n03x4               g132(.a(new_n226), .b(new_n227), .out0(\s[21] ));
  nor002aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  tech160nm_finand02aa1n03p5x5 g136(.a(\b[21] ), .b(\a[22] ), .o1(new_n232));
  oai022aa1n02x5               g137(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n232), .b(new_n233), .out0(new_n234));
  nand02aa1n02x5               g139(.a(new_n226), .b(new_n234), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n231), .c(new_n230), .d(new_n226), .o1(\s[22] ));
  nand42aa1n03x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nor042aa1n02x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nano23aa1n03x7               g143(.a(new_n229), .b(new_n238), .c(new_n232), .d(new_n237), .out0(new_n239));
  norb02aa1n02x7               g144(.a(new_n239), .b(new_n221), .out0(new_n240));
  aoai13aa1n04x5               g145(.a(new_n240), .b(new_n190), .c(new_n148), .d(new_n185), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n239), .b(new_n223), .c(new_n220), .d(new_n205), .o1(new_n242));
  oai012aa1n02x5               g147(.a(new_n232), .b(new_n238), .c(new_n229), .o1(new_n243));
  and002aa1n02x5               g148(.a(new_n242), .b(new_n243), .o(new_n244));
  xorc02aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xnbna2aa1n03x5               g150(.a(new_n245), .b(new_n241), .c(new_n244), .out0(\s[23] ));
  norp02aa1n02x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aob012aa1n03x5               g153(.a(new_n245), .b(new_n241), .c(new_n244), .out0(new_n249));
  xorc02aa1n12x5               g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  nanp02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  oai022aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n251), .b(new_n252), .out0(new_n253));
  nand02aa1n04x5               g158(.a(new_n249), .b(new_n253), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n250), .c(new_n248), .d(new_n249), .o1(\s[24] ));
  nano32aa1n02x5               g160(.a(new_n221), .b(new_n250), .c(new_n239), .d(new_n245), .out0(new_n256));
  tech160nm_fixnrc02aa1n02p5x5 g161(.a(\b[22] ), .b(\a[23] ), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n250), .b(new_n257), .out0(new_n258));
  inv000aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n252), .b(new_n251), .o1(new_n260));
  aoai13aa1n12x5               g165(.a(new_n260), .b(new_n259), .c(new_n242), .d(new_n243), .o1(new_n261));
  xorc02aa1n06x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n261), .c(new_n191), .d(new_n256), .o1(new_n263));
  aoi112aa1n02x7               g168(.a(new_n262), .b(new_n261), .c(new_n191), .d(new_n256), .o1(new_n264));
  norb02aa1n03x4               g169(.a(new_n263), .b(new_n264), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  tech160nm_fixorc02aa1n03p5x5 g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  nanp02aa1n02x5               g173(.a(\b[25] ), .b(\a[26] ), .o1(new_n269));
  oai022aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n269), .b(new_n270), .out0(new_n271));
  nand02aa1n02x5               g176(.a(new_n263), .b(new_n271), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n268), .c(new_n267), .d(new_n263), .o1(\s[26] ));
  and002aa1n02x5               g178(.a(new_n268), .b(new_n262), .o(new_n274));
  nano32aa1n03x7               g179(.a(new_n221), .b(new_n274), .c(new_n239), .d(new_n258), .out0(new_n275));
  aoai13aa1n12x5               g180(.a(new_n275), .b(new_n190), .c(new_n148), .d(new_n185), .o1(new_n276));
  aoi022aa1n09x5               g181(.a(new_n261), .b(new_n274), .c(new_n269), .d(new_n270), .o1(new_n277));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n277), .c(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  nanp02aa1n02x5               g186(.a(new_n261), .b(new_n274), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(new_n270), .b(new_n269), .o1(new_n283));
  nand23aa1n06x5               g188(.a(new_n282), .b(new_n276), .c(new_n283), .o1(new_n284));
  nanp02aa1n03x5               g189(.a(new_n284), .b(new_n278), .o1(new_n285));
  tech160nm_fixorc02aa1n02p5x5 g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n278), .o1(new_n287));
  oai022aa1d24x5               g192(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(\a[28] ), .c(\b[27] ), .o1(new_n289));
  aoai13aa1n02x7               g194(.a(new_n289), .b(new_n287), .c(new_n277), .d(new_n276), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n286), .c(new_n285), .d(new_n281), .o1(\s[28] ));
  and002aa1n02x5               g196(.a(new_n286), .b(new_n278), .o(new_n292));
  inv000aa1n02x5               g197(.a(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(\b[27] ), .o1(new_n294));
  oaib12aa1n09x5               g199(.a(new_n288), .b(new_n294), .c(\a[28] ), .out0(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  norb02aa1n02x5               g201(.a(new_n295), .b(new_n296), .out0(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n293), .c(new_n277), .d(new_n276), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n295), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n296), .b(new_n299), .c(new_n284), .d(new_n292), .o1(new_n300));
  nanp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[29] ));
  nanp02aa1n02x5               g206(.a(\b[0] ), .b(\a[1] ), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g208(.a(new_n296), .b(new_n278), .c(new_n286), .out0(new_n304));
  tech160nm_fioaoi03aa1n03p5x5 g209(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n305), .c(new_n284), .d(new_n304), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n305), .b(new_n306), .o1(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n308), .c(new_n277), .d(new_n276), .o1(new_n310));
  nanp02aa1n03x5               g215(.a(new_n307), .b(new_n310), .o1(\s[30] ));
  nano23aa1n02x4               g216(.a(new_n306), .b(new_n296), .c(new_n286), .d(new_n278), .out0(new_n312));
  nanp02aa1n03x5               g217(.a(new_n284), .b(new_n312), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[31] ), .b(\b[30] ), .out0(new_n314));
  inv000aa1n02x5               g219(.a(new_n312), .o1(new_n315));
  oai012aa1n02x5               g220(.a(new_n314), .b(\b[29] ), .c(\a[30] ), .o1(new_n316));
  aoib12aa1n02x5               g221(.a(new_n316), .b(new_n305), .c(new_n306), .out0(new_n317));
  aoai13aa1n02x7               g222(.a(new_n317), .b(new_n315), .c(new_n277), .d(new_n276), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[30] ), .b(\b[29] ), .c(new_n309), .carry(new_n319));
  aoai13aa1n03x5               g224(.a(new_n318), .b(new_n314), .c(new_n313), .d(new_n319), .o1(\s[31] ));
  norb02aa1n02x5               g225(.a(new_n98), .b(new_n99), .out0(new_n321));
  xobna2aa1n03x5               g226(.a(new_n321), .b(new_n101), .c(new_n97), .out0(\s[3] ));
  inv000aa1d42x5               g227(.a(new_n99), .o1(new_n323));
  nanp03aa1n02x5               g228(.a(new_n321), .b(new_n97), .c(new_n101), .o1(new_n324));
  xnbna2aa1n03x5               g229(.a(new_n104), .b(new_n324), .c(new_n323), .out0(\s[4] ));
  xnbna2aa1n03x5               g230(.a(new_n113), .b(new_n105), .c(new_n106), .out0(\s[5] ));
  nanp02aa1n02x5               g231(.a(new_n107), .b(new_n113), .o1(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n110), .b(new_n327), .c(new_n122), .out0(\s[6] ));
  inv000aa1d42x5               g233(.a(new_n117), .o1(new_n329));
  aoi012aa1n02x5               g234(.a(new_n123), .b(new_n107), .c(new_n146), .o1(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n118), .out0(\s[7] ));
  orn002aa1n02x5               g236(.a(new_n330), .b(new_n119), .o(new_n332));
  xnbna2aa1n03x5               g237(.a(new_n116), .b(new_n332), .c(new_n329), .out0(\s[8] ));
  xorb03aa1n02x5               g238(.a(new_n148), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


