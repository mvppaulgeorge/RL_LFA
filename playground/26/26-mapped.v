// Benchmark "adder" written by ABC on Thu Jul 18 01:30:35 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n333, new_n334, new_n335, new_n336, new_n337,
    new_n338, new_n340, new_n341, new_n342, new_n343, new_n344, new_n346,
    new_n348, new_n349, new_n350, new_n352, new_n354, new_n356;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nand02aa1d24x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  inv000aa1n04x5               g003(.a(new_n98), .o1(new_n99));
  oaoi03aa1n12x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1d28x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1d12x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n06x5               g009(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n105));
  nand02aa1n04x5               g010(.a(new_n105), .b(new_n100), .o1(new_n106));
  aoi012aa1n09x5               g011(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n107));
  nor002aa1n16x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1n20x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1n04x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n20x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nano23aa1n06x5               g016(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  nor042aa1d18x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1d24x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nand23aa1n03x5               g021(.a(new_n112), .b(new_n113), .c(new_n116), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n114), .b(new_n109), .c(new_n115), .out0(new_n118));
  oai022aa1n04x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  inv030aa1n02x5               g024(.a(new_n114), .o1(new_n120));
  oaoi03aa1n09x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  aoi013aa1n06x4               g026(.a(new_n121), .b(new_n118), .c(new_n113), .d(new_n119), .o1(new_n122));
  aoai13aa1n12x5               g027(.a(new_n122), .b(new_n117), .c(new_n106), .d(new_n107), .o1(new_n123));
  nand42aa1n03x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nand42aa1n02x5               g029(.a(new_n97), .b(new_n124), .o1(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(new_n123), .b(new_n126), .o1(new_n127));
  norp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n03x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n132));
  nano32aa1n03x7               g037(.a(new_n128), .b(new_n97), .c(new_n129), .d(new_n124), .out0(new_n133));
  tech160nm_fixorc02aa1n03p5x5 g038(.a(\a[11] ), .b(\b[10] ), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n132), .c(new_n123), .d(new_n133), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n132), .b(new_n134), .c(new_n123), .d(new_n133), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  nor022aa1n08x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  tech160nm_fixorc02aa1n03p5x5 g044(.a(\a[12] ), .b(\b[11] ), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n135), .c(new_n139), .out0(\s[12] ));
  nano32aa1n03x7               g046(.a(new_n125), .b(new_n140), .c(new_n130), .d(new_n134), .out0(new_n142));
  nanp02aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nor042aa1n03x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanb03aa1n02x5               g050(.a(new_n144), .b(new_n145), .c(new_n143), .out0(new_n146));
  nanb03aa1n12x5               g051(.a(new_n146), .b(new_n132), .c(new_n139), .out0(new_n147));
  tech160nm_fiaoi012aa1n04x5   g052(.a(new_n144), .b(new_n138), .c(new_n145), .o1(new_n148));
  nand42aa1n03x5               g053(.a(new_n147), .b(new_n148), .o1(new_n149));
  norp02aa1n24x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1n03x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n149), .c(new_n123), .d(new_n142), .o1(new_n154));
  aoi112aa1n02x5               g059(.a(new_n153), .b(new_n149), .c(new_n123), .d(new_n142), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(\s[13] ));
  orn002aa1n02x5               g061(.a(\a[13] ), .b(\b[12] ), .o(new_n157));
  nor002aa1n16x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n06x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  xnbna2aa1n03x5               g065(.a(new_n160), .b(new_n154), .c(new_n157), .out0(\s[14] ));
  nano23aa1n06x5               g066(.a(new_n150), .b(new_n158), .c(new_n159), .d(new_n151), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n149), .c(new_n123), .d(new_n142), .o1(new_n163));
  oai012aa1n02x5               g068(.a(new_n159), .b(new_n158), .c(new_n150), .o1(new_n164));
  xorc02aa1n12x5               g069(.a(\a[15] ), .b(\b[14] ), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n163), .c(new_n164), .out0(\s[15] ));
  aob012aa1n03x5               g071(.a(new_n165), .b(new_n163), .c(new_n164), .out0(new_n167));
  tech160nm_fixorc02aa1n04x5   g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  inv000aa1d42x5               g073(.a(\a[15] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(\b[14] ), .o1(new_n170));
  inv000aa1d42x5               g075(.a(\a[16] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[15] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n171), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  aoi022aa1n02x5               g079(.a(new_n173), .b(new_n174), .c(new_n170), .d(new_n169), .o1(new_n175));
  nor042aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  inv000aa1n03x5               g081(.a(new_n176), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n165), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n177), .b(new_n178), .c(new_n163), .d(new_n164), .o1(new_n179));
  aoi022aa1n03x5               g084(.a(new_n179), .b(new_n168), .c(new_n167), .d(new_n175), .o1(\s[16] ));
  nand23aa1n09x5               g085(.a(new_n162), .b(new_n165), .c(new_n168), .o1(new_n181));
  nano32aa1d12x5               g086(.a(new_n181), .b(new_n133), .c(new_n134), .d(new_n140), .out0(new_n182));
  nand42aa1n06x5               g087(.a(new_n123), .b(new_n182), .o1(new_n183));
  inv040aa1n02x5               g088(.a(new_n181), .o1(new_n184));
  oai112aa1n02x5               g089(.a(new_n173), .b(new_n174), .c(new_n170), .d(new_n169), .o1(new_n185));
  oai112aa1n02x7               g090(.a(new_n177), .b(new_n159), .c(new_n158), .d(new_n150), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n171), .b(new_n172), .c(new_n176), .o1(new_n187));
  oa0012aa1n12x5               g092(.a(new_n187), .b(new_n186), .c(new_n185), .o(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  aoi012aa1n12x5               g094(.a(new_n189), .b(new_n149), .c(new_n184), .o1(new_n190));
  nand22aa1n12x5               g095(.a(new_n183), .b(new_n190), .o1(new_n191));
  xorc02aa1n12x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  aoi112aa1n02x5               g097(.a(new_n189), .b(new_n192), .c(new_n149), .d(new_n184), .o1(new_n193));
  aoi022aa1n02x5               g098(.a(new_n191), .b(new_n192), .c(new_n183), .d(new_n193), .o1(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(\b[16] ), .b(new_n195), .out0(new_n196));
  aoai13aa1n12x5               g101(.a(new_n188), .b(new_n181), .c(new_n147), .d(new_n148), .o1(new_n197));
  aoai13aa1n03x5               g102(.a(new_n192), .b(new_n197), .c(new_n123), .d(new_n182), .o1(new_n198));
  nor042aa1d18x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand42aa1d28x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  norb02aa1n06x4               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n198), .c(new_n196), .out0(\s[18] ));
  and002aa1n02x5               g107(.a(new_n192), .b(new_n201), .o(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n197), .c(new_n123), .d(new_n182), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  nor002aa1d32x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand42aa1n08x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  norb02aa1n12x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  xnbna2aa1n03x5               g114(.a(new_n209), .b(new_n204), .c(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g116(.a(new_n209), .b(new_n205), .c(new_n191), .d(new_n203), .o1(new_n212));
  norp02aa1n24x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand02aa1d28x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  norb02aa1n03x4               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  inv000aa1d42x5               g120(.a(\a[19] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[18] ), .o1(new_n217));
  aboi22aa1n03x5               g122(.a(new_n213), .b(new_n214), .c(new_n216), .d(new_n217), .out0(new_n218));
  inv000aa1n09x5               g123(.a(new_n207), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n209), .o1(new_n220));
  aoai13aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n204), .d(new_n206), .o1(new_n221));
  aoi022aa1n03x5               g126(.a(new_n221), .b(new_n215), .c(new_n212), .d(new_n218), .o1(\s[20] ));
  nano32aa1n03x7               g127(.a(new_n220), .b(new_n192), .c(new_n215), .d(new_n201), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n197), .c(new_n123), .d(new_n182), .o1(new_n224));
  nanb03aa1n12x5               g129(.a(new_n213), .b(new_n214), .c(new_n208), .out0(new_n225));
  nor042aa1n09x5               g130(.a(\b[16] ), .b(\a[17] ), .o1(new_n226));
  oai112aa1n06x5               g131(.a(new_n219), .b(new_n200), .c(new_n199), .d(new_n226), .o1(new_n227));
  aoi012aa1d18x5               g132(.a(new_n213), .b(new_n207), .c(new_n214), .o1(new_n228));
  oai012aa1d24x5               g133(.a(new_n228), .b(new_n227), .c(new_n225), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  nor002aa1d32x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nand42aa1n06x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  norb02aa1n12x5               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  xnbna2aa1n03x5               g138(.a(new_n233), .b(new_n224), .c(new_n230), .out0(\s[21] ));
  aoai13aa1n06x5               g139(.a(new_n233), .b(new_n229), .c(new_n191), .d(new_n223), .o1(new_n235));
  nor042aa1n06x5               g140(.a(\b[21] ), .b(\a[22] ), .o1(new_n236));
  nand42aa1n16x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  aoib12aa1n02x5               g143(.a(new_n231), .b(new_n237), .c(new_n236), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n231), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n233), .o1(new_n241));
  aoai13aa1n03x5               g146(.a(new_n240), .b(new_n241), .c(new_n224), .d(new_n230), .o1(new_n242));
  aoi022aa1n03x5               g147(.a(new_n242), .b(new_n238), .c(new_n235), .d(new_n239), .o1(\s[22] ));
  inv020aa1n02x5               g148(.a(new_n223), .o1(new_n244));
  nano22aa1n03x7               g149(.a(new_n244), .b(new_n233), .c(new_n238), .out0(new_n245));
  aoai13aa1n02x7               g150(.a(new_n245), .b(new_n197), .c(new_n123), .d(new_n182), .o1(new_n246));
  nano22aa1n03x5               g151(.a(new_n213), .b(new_n208), .c(new_n214), .out0(new_n247));
  oai012aa1n02x7               g152(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .o1(new_n248));
  oab012aa1n02x4               g153(.a(new_n248), .b(new_n226), .c(new_n199), .out0(new_n249));
  inv020aa1n02x5               g154(.a(new_n228), .o1(new_n250));
  nano23aa1d15x5               g155(.a(new_n231), .b(new_n236), .c(new_n237), .d(new_n232), .out0(new_n251));
  aoai13aa1n04x5               g156(.a(new_n251), .b(new_n250), .c(new_n249), .d(new_n247), .o1(new_n252));
  aoi012aa1d18x5               g157(.a(new_n236), .b(new_n231), .c(new_n237), .o1(new_n253));
  nand42aa1n03x5               g158(.a(new_n252), .b(new_n253), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n191), .d(new_n245), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n253), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n255), .b(new_n257), .c(new_n229), .d(new_n251), .o1(new_n258));
  aobi12aa1n02x7               g163(.a(new_n256), .b(new_n258), .c(new_n246), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  nor042aa1n06x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  norp02aa1n02x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n254), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n261), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n255), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n264), .b(new_n265), .c(new_n246), .d(new_n263), .o1(new_n266));
  aoi022aa1n03x5               g171(.a(new_n266), .b(new_n260), .c(new_n256), .d(new_n262), .o1(\s[24] ));
  and002aa1n06x5               g172(.a(new_n260), .b(new_n255), .o(new_n268));
  nano22aa1n03x7               g173(.a(new_n244), .b(new_n268), .c(new_n251), .out0(new_n269));
  aoai13aa1n02x7               g174(.a(new_n269), .b(new_n197), .c(new_n123), .d(new_n182), .o1(new_n270));
  inv000aa1n06x5               g175(.a(new_n268), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .c(new_n264), .carry(new_n272));
  aoai13aa1n12x5               g177(.a(new_n272), .b(new_n271), .c(new_n252), .d(new_n253), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[25] ), .b(\b[24] ), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n273), .c(new_n191), .d(new_n269), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n268), .b(new_n257), .c(new_n229), .d(new_n251), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n274), .o1(new_n277));
  and003aa1n02x5               g182(.a(new_n276), .b(new_n277), .c(new_n272), .o(new_n278));
  aobi12aa1n03x7               g183(.a(new_n275), .b(new_n278), .c(new_n270), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g184(.a(\a[26] ), .b(\b[25] ), .out0(new_n280));
  nor042aa1n06x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  norp02aa1n02x5               g186(.a(new_n280), .b(new_n281), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n273), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n281), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n277), .c(new_n270), .d(new_n283), .o1(new_n285));
  aoi022aa1n03x5               g190(.a(new_n285), .b(new_n280), .c(new_n275), .d(new_n282), .o1(\s[26] ));
  and002aa1n12x5               g191(.a(new_n280), .b(new_n274), .o(new_n287));
  nano32aa1n03x7               g192(.a(new_n244), .b(new_n287), .c(new_n251), .d(new_n268), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n288), .b(new_n197), .c(new_n123), .d(new_n182), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n287), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n290), .c(new_n276), .d(new_n272), .o1(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n291), .o1(new_n295));
  aoi112aa1n02x5               g200(.a(new_n293), .b(new_n295), .c(new_n273), .d(new_n287), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n294), .b(new_n296), .c(new_n289), .out0(\s[27] ));
  xorc02aa1n02x5               g202(.a(\a[28] ), .b(\b[27] ), .out0(new_n298));
  norp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n298), .b(new_n299), .o1(new_n300));
  tech160nm_fiaoi012aa1n05x5   g205(.a(new_n295), .b(new_n273), .c(new_n287), .o1(new_n301));
  inv000aa1n03x5               g206(.a(new_n299), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n293), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n302), .b(new_n303), .c(new_n301), .d(new_n289), .o1(new_n304));
  aoi022aa1n03x5               g209(.a(new_n304), .b(new_n298), .c(new_n294), .d(new_n300), .o1(\s[28] ));
  and002aa1n02x5               g210(.a(new_n298), .b(new_n293), .o(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[29] ), .b(\b[28] ), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n309));
  norb02aa1n02x5               g214(.a(new_n309), .b(new_n308), .out0(new_n310));
  inv000aa1d42x5               g215(.a(new_n306), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n309), .b(new_n311), .c(new_n301), .d(new_n289), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n312), .b(new_n308), .c(new_n307), .d(new_n310), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g219(.a(new_n303), .b(new_n298), .c(new_n308), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n315), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[30] ), .b(\b[29] ), .out0(new_n317));
  oao003aa1n02x5               g222(.a(\a[29] ), .b(\b[28] ), .c(new_n309), .carry(new_n318));
  norb02aa1n02x5               g223(.a(new_n318), .b(new_n317), .out0(new_n319));
  inv000aa1n02x5               g224(.a(new_n315), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n318), .b(new_n320), .c(new_n301), .d(new_n289), .o1(new_n321));
  aoi022aa1n03x5               g226(.a(new_n321), .b(new_n317), .c(new_n316), .d(new_n319), .o1(\s[30] ));
  nano32aa1d12x5               g227(.a(new_n303), .b(new_n317), .c(new_n298), .d(new_n308), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n292), .c(new_n191), .d(new_n288), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  and002aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .o(new_n326));
  oabi12aa1n02x5               g231(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .out0(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n318), .c(new_n326), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n323), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .c(new_n318), .carry(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n329), .c(new_n301), .d(new_n289), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n331), .b(new_n325), .c(new_n324), .d(new_n328), .o1(\s[31] ));
  inv000aa1d42x5               g237(.a(\b[1] ), .o1(new_n333));
  inv000aa1d42x5               g238(.a(\a[2] ), .o1(new_n334));
  aoi022aa1n02x5               g239(.a(new_n333), .b(new_n334), .c(\a[1] ), .d(\b[0] ), .o1(new_n335));
  oaib12aa1n02x5               g240(.a(new_n335), .b(new_n333), .c(\a[2] ), .out0(new_n336));
  norb02aa1n02x5               g241(.a(new_n104), .b(new_n103), .out0(new_n337));
  aboi22aa1n03x5               g242(.a(new_n103), .b(new_n104), .c(new_n334), .d(new_n333), .out0(new_n338));
  aoi022aa1n02x5               g243(.a(new_n336), .b(new_n338), .c(new_n100), .d(new_n337), .o1(\s[3] ));
  oaoi03aa1n02x5               g244(.a(new_n334), .b(new_n333), .c(new_n98), .o1(new_n340));
  nona23aa1n02x4               g245(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n341));
  oai012aa1n06x5               g246(.a(new_n107), .b(new_n341), .c(new_n340), .o1(new_n342));
  obai22aa1n02x7               g247(.a(new_n102), .b(new_n101), .c(\a[3] ), .d(\b[2] ), .out0(new_n343));
  aoi012aa1n02x5               g248(.a(new_n343), .b(new_n100), .c(new_n337), .o1(new_n344));
  oaoi13aa1n02x5               g249(.a(new_n344), .b(new_n342), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  norb02aa1n02x5               g250(.a(new_n111), .b(new_n110), .out0(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n346), .b(new_n106), .c(new_n107), .out0(\s[5] ));
  norb02aa1n02x5               g252(.a(new_n109), .b(new_n108), .out0(new_n348));
  aoai13aa1n02x5               g253(.a(new_n348), .b(new_n110), .c(new_n342), .d(new_n111), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n110), .b(new_n348), .c(new_n342), .d(new_n111), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n349), .b(new_n350), .out0(\s[6] ));
  inv000aa1d42x5               g256(.a(new_n108), .o1(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n116), .b(new_n349), .c(new_n352), .out0(\s[7] ));
  aob012aa1n03x5               g258(.a(new_n116), .b(new_n349), .c(new_n352), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n113), .b(new_n354), .c(new_n120), .out0(\s[8] ));
  nanb02aa1n02x5               g260(.a(new_n117), .b(new_n342), .out0(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n126), .b(new_n356), .c(new_n122), .out0(\s[9] ));
endmodule

