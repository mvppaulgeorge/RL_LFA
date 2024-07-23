// Benchmark "adder" written by ABC on Thu Jul 18 02:43:05 2024

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
    new_n133, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n315, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n12x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand22aa1n09x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n06x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nona23aa1d18x5               g011(.a(new_n105), .b(new_n104), .c(new_n106), .d(new_n103), .out0(new_n107));
  ao0012aa1n03x7               g012(.a(new_n106), .b(new_n103), .c(new_n105), .o(new_n108));
  oabi12aa1n18x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .out0(new_n109));
  oai022aa1d18x5               g014(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n110));
  nor002aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  aob012aa1n02x5               g021(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(new_n117));
  nor043aa1n03x5               g022(.a(new_n115), .b(new_n117), .c(new_n110), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n114), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n111), .b(new_n113), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n110), .b(new_n116), .o1(new_n121));
  oai112aa1n06x5               g026(.a(new_n119), .b(new_n120), .c(new_n115), .d(new_n121), .o1(new_n122));
  and002aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o(new_n123));
  norp02aa1n02x5               g028(.a(new_n123), .b(new_n97), .o1(new_n124));
  aoai13aa1n03x5               g029(.a(new_n124), .b(new_n122), .c(new_n109), .d(new_n118), .o1(new_n125));
  nor042aa1d18x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1d28x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x7               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  nor042aa1d18x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n04x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n125), .b(new_n126), .c(new_n97), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n132), .b(new_n133), .c(new_n127), .out0(\s[11] ));
  aoi013aa1n03x5               g039(.a(new_n130), .b(new_n133), .c(new_n131), .d(new_n127), .o1(new_n135));
  xnrb03aa1n03x5               g040(.a(new_n135), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  inv040aa1n03x5               g041(.a(new_n130), .o1(new_n137));
  nor022aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  tech160nm_finand02aa1n03p5x5 g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb03aa1n12x5               g044(.a(new_n138), .b(new_n139), .c(new_n131), .out0(new_n140));
  nano32aa1n02x4               g045(.a(new_n140), .b(new_n124), .c(new_n128), .d(new_n137), .out0(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n122), .c(new_n109), .d(new_n118), .o1(new_n142));
  oai112aa1n06x5               g047(.a(new_n137), .b(new_n127), .c(new_n126), .d(new_n97), .o1(new_n143));
  oaoi03aa1n09x5               g048(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n144));
  oabi12aa1n18x5               g049(.a(new_n144), .b(new_n143), .c(new_n140), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n142), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g053(.a(\a[13] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(\b[12] ), .o1(new_n150));
  oaoi03aa1n03x5               g055(.a(new_n149), .b(new_n150), .c(new_n147), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1d28x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nor002aa1n06x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand42aa1n16x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nano23aa1d15x5               g061(.a(new_n153), .b(new_n155), .c(new_n156), .d(new_n154), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  oa0012aa1n12x5               g063(.a(new_n156), .b(new_n155), .c(new_n153), .o(new_n159));
  inv000aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n06x5               g065(.a(new_n160), .b(new_n158), .c(new_n142), .d(new_n146), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nand02aa1n03x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nor042aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nand02aa1n03x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n163), .b(new_n167), .c(new_n161), .d(new_n164), .o1(new_n169));
  norb02aa1n03x4               g074(.a(new_n168), .b(new_n169), .out0(\s[16] ));
  nona32aa1n03x5               g075(.a(new_n128), .b(new_n123), .c(new_n130), .d(new_n97), .out0(new_n171));
  nano22aa1n03x7               g076(.a(new_n138), .b(new_n131), .c(new_n139), .out0(new_n172));
  nona23aa1d24x5               g077(.a(new_n166), .b(new_n164), .c(new_n163), .d(new_n165), .out0(new_n173));
  nano23aa1d15x5               g078(.a(new_n171), .b(new_n173), .c(new_n157), .d(new_n172), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n122), .c(new_n109), .d(new_n118), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n173), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n176), .b(new_n159), .c(new_n145), .d(new_n157), .o1(new_n177));
  tech160nm_fiaoi012aa1n03p5x5 g082(.a(new_n165), .b(new_n163), .c(new_n166), .o1(new_n178));
  nand23aa1n06x5               g083(.a(new_n175), .b(new_n177), .c(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g085(.a(\a[18] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  nand42aa1n04x5               g090(.a(new_n109), .b(new_n118), .o1(new_n186));
  nanb02aa1n12x5               g091(.a(new_n122), .b(new_n186), .out0(new_n187));
  oai012aa1n02x5               g092(.a(new_n127), .b(\b[10] ), .c(\a[11] ), .o1(new_n188));
  oab012aa1n02x4               g093(.a(new_n188), .b(new_n97), .c(new_n126), .out0(new_n189));
  aoai13aa1n04x5               g094(.a(new_n157), .b(new_n144), .c(new_n189), .d(new_n172), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n178), .b(new_n173), .c(new_n190), .d(new_n160), .o1(new_n191));
  xroi22aa1d06x4               g096(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n191), .c(new_n187), .d(new_n174), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[17] ), .o1(new_n194));
  oai022aa1d24x5               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  oa0012aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n181), .o(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  xorc02aa1n12x5               g102(.a(\a[19] ), .b(\b[18] ), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n193), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g105(.a(\a[19] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\b[18] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n198), .b(new_n196), .c(new_n179), .d(new_n192), .o1(new_n204));
  xnrc02aa1n12x5               g109(.a(\b[19] ), .b(\a[20] ), .out0(new_n205));
  tech160nm_fiaoi012aa1n02p5x5 g110(.a(new_n205), .b(new_n204), .c(new_n203), .o1(new_n206));
  inv040aa1n03x5               g111(.a(new_n198), .o1(new_n207));
  tech160nm_fiaoi012aa1n02p5x5 g112(.a(new_n207), .b(new_n193), .c(new_n197), .o1(new_n208));
  nano22aa1n02x4               g113(.a(new_n208), .b(new_n203), .c(new_n205), .out0(new_n209));
  nor002aa1n02x5               g114(.a(new_n206), .b(new_n209), .o1(\s[20] ));
  nona22aa1n09x5               g115(.a(new_n192), .b(new_n207), .c(new_n205), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n191), .c(new_n187), .d(new_n174), .o1(new_n213));
  oai122aa1n12x5               g118(.a(new_n195), .b(new_n201), .c(new_n202), .d(new_n181), .e(new_n194), .o1(new_n214));
  oa0022aa1n06x5               g119(.a(\a[20] ), .b(\b[19] ), .c(\a[19] ), .d(\b[18] ), .o(new_n215));
  aoi022aa1d24x5               g120(.a(new_n214), .b(new_n215), .c(\b[19] ), .d(\a[20] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  xnbna2aa1n03x5               g123(.a(new_n218), .b(new_n213), .c(new_n217), .out0(\s[21] ));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  nanb02aa1n02x5               g125(.a(\b[20] ), .b(new_n220), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n218), .b(new_n216), .c(new_n179), .d(new_n212), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoi012aa1n03x5               g128(.a(new_n223), .b(new_n222), .c(new_n221), .o1(new_n224));
  aobi12aa1n06x5               g129(.a(new_n218), .b(new_n213), .c(new_n217), .out0(new_n225));
  nano22aa1n02x4               g130(.a(new_n225), .b(new_n221), .c(new_n223), .out0(new_n226));
  nor002aa1n02x5               g131(.a(new_n224), .b(new_n226), .o1(\s[22] ));
  inv000aa1d42x5               g132(.a(\a[23] ), .o1(new_n228));
  norb02aa1n06x4               g133(.a(new_n218), .b(new_n223), .out0(new_n229));
  norb02aa1n06x5               g134(.a(new_n229), .b(new_n211), .out0(new_n230));
  oaoi03aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n221), .o1(new_n231));
  tech160nm_fiao0012aa1n02p5x5 g136(.a(new_n231), .b(new_n216), .c(new_n229), .o(new_n232));
  tech160nm_fiaoi012aa1n05x5   g137(.a(new_n232), .b(new_n179), .c(new_n230), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(new_n228), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  and002aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o(new_n236));
  aoai13aa1n03x5               g141(.a(new_n230), .b(new_n191), .c(new_n187), .d(new_n174), .o1(new_n237));
  nona32aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n236), .d(new_n235), .out0(new_n238));
  oaib12aa1n03x5               g143(.a(new_n238), .b(new_n228), .c(\b[22] ), .out0(new_n239));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  nanp02aa1n03x5               g145(.a(new_n239), .b(new_n240), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n238), .b(new_n240), .c(new_n236), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n241), .b(new_n242), .o1(\s[24] ));
  inv000aa1d42x5               g148(.a(\a[24] ), .o1(new_n244));
  xroi22aa1d04x5               g149(.a(new_n228), .b(\b[22] ), .c(new_n244), .d(\b[23] ), .out0(new_n245));
  nano22aa1n03x7               g150(.a(new_n211), .b(new_n229), .c(new_n245), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n191), .c(new_n187), .d(new_n174), .o1(new_n247));
  nand23aa1n03x5               g152(.a(new_n216), .b(new_n229), .c(new_n245), .o1(new_n248));
  norp02aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n250));
  aoi112aa1n06x5               g155(.a(new_n250), .b(new_n249), .c(new_n245), .d(new_n231), .o1(new_n251));
  nand22aa1n04x5               g156(.a(new_n251), .b(new_n248), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  xnrc02aa1n12x5               g158(.a(\b[24] ), .b(\a[25] ), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  xnbna2aa1n03x5               g160(.a(new_n255), .b(new_n247), .c(new_n253), .out0(\s[25] ));
  nor042aa1n03x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n255), .b(new_n252), .c(new_n179), .d(new_n246), .o1(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[25] ), .b(\a[26] ), .out0(new_n260));
  tech160nm_fiaoi012aa1n02p5x5 g165(.a(new_n260), .b(new_n259), .c(new_n258), .o1(new_n261));
  aoi012aa1n03x5               g166(.a(new_n254), .b(new_n247), .c(new_n253), .o1(new_n262));
  nano22aa1n02x4               g167(.a(new_n262), .b(new_n258), .c(new_n260), .out0(new_n263));
  norp02aa1n03x5               g168(.a(new_n261), .b(new_n263), .o1(\s[26] ));
  nor042aa1n03x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n266), .b(new_n265), .out0(new_n267));
  nor042aa1n03x5               g172(.a(new_n260), .b(new_n254), .o1(new_n268));
  nano32aa1n03x7               g173(.a(new_n211), .b(new_n268), .c(new_n229), .d(new_n245), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n191), .c(new_n187), .d(new_n174), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n268), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n258), .carry(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n251), .d(new_n248), .o1(new_n273));
  inv030aa1n03x5               g178(.a(new_n273), .o1(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n267), .b(new_n270), .c(new_n274), .out0(\s[27] ));
  inv000aa1d42x5               g180(.a(new_n265), .o1(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoi022aa1n02x7               g182(.a(new_n270), .b(new_n274), .c(\b[26] ), .d(\a[27] ), .o1(new_n278));
  nano22aa1n03x5               g183(.a(new_n278), .b(new_n276), .c(new_n277), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n266), .b(new_n273), .c(new_n179), .d(new_n269), .o1(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n277), .b(new_n280), .c(new_n276), .o1(new_n281));
  norp02aa1n03x5               g186(.a(new_n281), .b(new_n279), .o1(\s[28] ));
  nano22aa1n02x4               g187(.a(new_n277), .b(new_n276), .c(new_n266), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n273), .c(new_n179), .d(new_n269), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n02x7               g192(.a(new_n283), .b(new_n270), .c(new_n274), .out0(new_n288));
  nano22aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n267), .b(new_n286), .c(new_n277), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n273), .c(new_n179), .d(new_n269), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n292), .b(new_n270), .c(new_n274), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  norb03aa1n02x5               g204(.a(new_n283), .b(new_n295), .c(new_n286), .out0(new_n300));
  aobi12aa1n02x7               g205(.a(new_n300), .b(new_n270), .c(new_n274), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n02x4               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n300), .b(new_n273), .c(new_n179), .d(new_n269), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g216(.a(new_n109), .o1(new_n312));
  oaoi03aa1n02x5               g217(.a(\a[5] ), .b(\b[4] ), .c(new_n312), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g219(.a(\b[5] ), .b(\a[6] ), .o1(new_n315));
  aoi012aa1n02x5               g220(.a(new_n315), .b(new_n313), .c(new_n116), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g222(.a(\a[7] ), .b(\b[6] ), .c(new_n316), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n187), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


