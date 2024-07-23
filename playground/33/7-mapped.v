// Benchmark "adder" written by ABC on Thu Jul 18 04:53:56 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n321, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n343, new_n344,
    new_n345, new_n346, new_n348, new_n349, new_n351, new_n352, new_n354,
    new_n355, new_n356, new_n358, new_n359, new_n360, new_n362, new_n364,
    new_n365;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[4] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[3] ), .o1(new_n98));
  nand42aa1n06x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nor042aa1d18x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  inv040aa1n09x5               g005(.a(new_n100), .o1(new_n101));
  oai112aa1n06x5               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n97), .o1(new_n102));
  oai112aa1n06x5               g007(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n103));
  nor042aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  tech160nm_fiaoi012aa1n03p5x5 g009(.a(new_n104), .b(\a[2] ), .c(\b[1] ), .o1(new_n105));
  nanp02aa1n06x5               g010(.a(new_n105), .b(new_n103), .o1(new_n106));
  tech160nm_fioaoi03aa1n03p5x5 g011(.a(new_n97), .b(new_n98), .c(new_n100), .o1(new_n107));
  oai012aa1n18x5               g012(.a(new_n107), .b(new_n106), .c(new_n102), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nona23aa1n09x5               g017(.a(new_n109), .b(new_n112), .c(new_n111), .d(new_n110), .out0(new_n113));
  nor002aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand02aa1d24x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor042aa1d18x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  tech160nm_finand02aa1n05x5   g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nona23aa1n09x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  nor042aa1n04x5               g023(.a(new_n118), .b(new_n113), .o1(new_n119));
  nanp02aa1n03x5               g024(.a(new_n108), .b(new_n119), .o1(new_n120));
  nanb03aa1n06x5               g025(.a(new_n116), .b(new_n117), .c(new_n109), .out0(new_n121));
  norb02aa1n12x5               g026(.a(new_n115), .b(new_n114), .out0(new_n122));
  tech160nm_fioai012aa1n04x5   g027(.a(new_n122), .b(new_n111), .c(new_n110), .o1(new_n123));
  norp02aa1n02x5               g028(.a(new_n123), .b(new_n121), .o1(new_n124));
  nor022aa1n12x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  inv040aa1n03x5               g030(.a(new_n116), .o1(new_n126));
  oaoi03aa1n12x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .o1(new_n127));
  inv040aa1n02x5               g032(.a(new_n127), .o1(new_n128));
  nona23aa1n06x5               g033(.a(new_n120), .b(new_n128), .c(new_n125), .d(new_n124), .out0(new_n129));
  xnrc02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  nand02aa1n06x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n130), .b(new_n129), .c(new_n131), .out0(\s[10] ));
  nor002aa1d32x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nand22aa1n03x5               g039(.a(new_n129), .b(new_n131), .o1(new_n135));
  nand02aa1n02x5               g040(.a(new_n135), .b(new_n134), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  nanp02aa1n12x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nor022aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanb03aa1n02x5               g044(.a(new_n139), .b(new_n137), .c(new_n138), .out0(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n138), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n137), .b(new_n133), .c(new_n129), .d(new_n131), .o1(new_n142));
  aboi22aa1n03x5               g047(.a(new_n140), .b(new_n136), .c(new_n142), .d(new_n141), .out0(\s[11] ));
  aoib12aa1n06x5               g048(.a(new_n139), .b(new_n136), .c(new_n140), .out0(new_n144));
  nor002aa1d32x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n12x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb03aa1n02x5               g052(.a(new_n147), .b(new_n139), .c(new_n145), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n140), .c(new_n135), .d(new_n134), .o1(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n144), .c(new_n147), .d(new_n146), .o1(\s[12] ));
  oai012aa1n12x5               g055(.a(new_n128), .b(new_n123), .c(new_n121), .o1(new_n151));
  nona23aa1n03x5               g056(.a(new_n138), .b(new_n147), .c(new_n145), .d(new_n139), .out0(new_n152));
  nanb02aa1d24x5               g057(.a(new_n125), .b(new_n131), .out0(new_n153));
  nor043aa1n03x5               g058(.a(new_n152), .b(new_n153), .c(new_n130), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n151), .c(new_n108), .d(new_n119), .o1(new_n155));
  inv000aa1d42x5               g060(.a(\a[10] ), .o1(new_n156));
  inv040aa1d30x5               g061(.a(\b[9] ), .o1(new_n157));
  oai022aa1n12x5               g062(.a(new_n156), .b(new_n157), .c(\b[10] ), .d(\a[11] ), .o1(new_n158));
  nanb03aa1d24x5               g063(.a(new_n145), .b(new_n147), .c(new_n138), .out0(new_n159));
  norp02aa1n09x5               g064(.a(new_n133), .b(new_n125), .o1(new_n160));
  tech160nm_fioai012aa1n04x5   g065(.a(new_n147), .b(new_n145), .c(new_n139), .o1(new_n161));
  oai013aa1d12x5               g066(.a(new_n161), .b(new_n159), .c(new_n158), .d(new_n160), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nanp02aa1n03x5               g068(.a(new_n155), .b(new_n163), .o1(new_n164));
  nor002aa1d32x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand02aa1n06x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  norp03aa1n02x5               g072(.a(new_n159), .b(new_n160), .c(new_n158), .o1(new_n168));
  norb03aa1n02x5               g073(.a(new_n161), .b(new_n168), .c(new_n167), .out0(new_n169));
  aoi022aa1n02x5               g074(.a(new_n164), .b(new_n167), .c(new_n155), .d(new_n169), .o1(\s[13] ));
  aoi012aa1n02x5               g075(.a(new_n165), .b(new_n164), .c(new_n166), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n06x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  oaih22aa1d12x5               g079(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n175));
  aoi122aa1n02x5               g080(.a(new_n175), .b(\b[13] ), .c(\a[14] ), .d(new_n164), .e(new_n167), .o1(new_n176));
  oabi12aa1n02x5               g081(.a(new_n176), .b(new_n171), .c(new_n174), .out0(\s[14] ));
  nona23aa1d24x5               g082(.a(new_n173), .b(new_n166), .c(new_n165), .d(new_n172), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nanp02aa1n03x5               g084(.a(new_n164), .b(new_n179), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n172), .b(new_n165), .c(new_n173), .o1(new_n181));
  xorc02aa1n02x5               g086(.a(\a[15] ), .b(\b[14] ), .out0(new_n182));
  xnbna2aa1n03x5               g087(.a(new_n182), .b(new_n180), .c(new_n181), .out0(\s[15] ));
  aobi12aa1n03x5               g088(.a(new_n182), .b(new_n180), .c(new_n181), .out0(new_n184));
  inv040aa1d32x5               g089(.a(\a[15] ), .o1(new_n185));
  inv040aa1n15x5               g090(.a(\b[14] ), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n181), .b(new_n178), .c(new_n155), .d(new_n163), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n185), .b(new_n186), .c(new_n187), .o1(new_n188));
  nor042aa1n04x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand02aa1d28x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n03x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  tech160nm_finand02aa1n03p5x5 g096(.a(new_n186), .b(new_n185), .o1(new_n192));
  nanb03aa1n02x5               g097(.a(new_n189), .b(new_n192), .c(new_n190), .out0(new_n193));
  oai022aa1n02x5               g098(.a(new_n191), .b(new_n188), .c(new_n184), .d(new_n193), .o1(\s[16] ));
  aoi012aa1d18x5               g099(.a(new_n151), .b(new_n108), .c(new_n119), .o1(new_n195));
  nanp02aa1n04x5               g100(.a(\b[14] ), .b(\a[15] ), .o1(new_n196));
  nano32aa1d12x5               g101(.a(new_n178), .b(new_n191), .c(new_n192), .d(new_n196), .out0(new_n197));
  nand02aa1d04x5               g102(.a(new_n154), .b(new_n197), .o1(new_n198));
  norp02aa1n02x5               g103(.a(new_n195), .b(new_n198), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n190), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n196), .b(\b[15] ), .c(\a[16] ), .o1(new_n201));
  nanp03aa1n03x5               g106(.a(new_n175), .b(new_n192), .c(new_n173), .o1(new_n202));
  aoai13aa1n02x5               g107(.a(new_n190), .b(new_n189), .c(new_n185), .d(new_n186), .o1(new_n203));
  oai013aa1n03x4               g108(.a(new_n203), .b(new_n202), .c(new_n200), .d(new_n201), .o1(new_n204));
  aoi012aa1n12x5               g109(.a(new_n204), .b(new_n162), .c(new_n197), .o1(new_n205));
  oai012aa1n12x5               g110(.a(new_n205), .b(new_n195), .c(new_n198), .o1(new_n206));
  xorc02aa1n12x5               g111(.a(\a[17] ), .b(\b[16] ), .out0(new_n207));
  norp03aa1n02x5               g112(.a(new_n202), .b(new_n201), .c(new_n200), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n203), .out0(new_n209));
  aoi112aa1n02x5               g114(.a(new_n209), .b(new_n208), .c(new_n162), .d(new_n197), .o1(new_n210));
  aboi22aa1n03x5               g115(.a(new_n199), .b(new_n210), .c(new_n206), .d(new_n207), .out0(\s[17] ));
  nor042aa1n03x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nand42aa1n02x5               g118(.a(new_n206), .b(new_n207), .o1(new_n214));
  xorc02aa1n12x5               g119(.a(\a[18] ), .b(\b[17] ), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  oaih22aa1d12x5               g121(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n217));
  nanb03aa1n02x5               g122(.a(new_n217), .b(new_n214), .c(new_n216), .out0(new_n218));
  aoai13aa1n02x5               g123(.a(new_n218), .b(new_n215), .c(new_n213), .d(new_n214), .o1(\s[18] ));
  nand22aa1n12x5               g124(.a(new_n215), .b(new_n207), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  oaoi03aa1n12x5               g126(.a(\a[18] ), .b(\b[17] ), .c(new_n213), .o1(new_n222));
  tech160nm_fixorc02aa1n03p5x5 g127(.a(\a[19] ), .b(\b[18] ), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n222), .c(new_n206), .d(new_n221), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(new_n223), .b(new_n222), .c(new_n206), .d(new_n221), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g132(.a(\a[19] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[18] ), .o1(new_n229));
  nanp02aa1n04x5               g134(.a(new_n229), .b(new_n228), .o1(new_n230));
  nor042aa1n09x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nand42aa1d28x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  norb02aa1n02x7               g137(.a(new_n232), .b(new_n231), .out0(new_n233));
  nano22aa1n02x4               g138(.a(new_n231), .b(new_n230), .c(new_n232), .out0(new_n234));
  tech160nm_finand02aa1n05x5   g139(.a(new_n224), .b(new_n234), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n224), .d(new_n230), .o1(\s[20] ));
  nano22aa1n03x7               g141(.a(new_n220), .b(new_n223), .c(new_n233), .out0(new_n237));
  nand42aa1n02x5               g142(.a(\b[18] ), .b(\a[19] ), .o1(new_n238));
  nanb03aa1n06x5               g143(.a(new_n231), .b(new_n232), .c(new_n238), .out0(new_n239));
  nand03aa1n04x5               g144(.a(new_n217), .b(new_n230), .c(new_n216), .o1(new_n240));
  aoai13aa1n12x5               g145(.a(new_n232), .b(new_n231), .c(new_n228), .d(new_n229), .o1(new_n241));
  oaih12aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[21] ), .b(\b[20] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n242), .c(new_n206), .d(new_n237), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n243), .o1(new_n245));
  oai112aa1n02x5               g150(.a(new_n245), .b(new_n241), .c(new_n240), .d(new_n239), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n206), .c(new_n237), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n244), .b(new_n247), .out0(\s[21] ));
  inv000aa1d42x5               g153(.a(\a[21] ), .o1(new_n249));
  nanb02aa1n02x5               g154(.a(\b[20] ), .b(new_n249), .out0(new_n250));
  xorc02aa1n02x5               g155(.a(\a[22] ), .b(\b[21] ), .out0(new_n251));
  oai022aa1n02x5               g156(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(\a[22] ), .c(\b[21] ), .o1(new_n253));
  tech160nm_finand02aa1n03p5x5 g158(.a(new_n244), .b(new_n253), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n251), .c(new_n244), .d(new_n250), .o1(\s[22] ));
  nand22aa1n03x5               g160(.a(new_n251), .b(new_n243), .o1(new_n256));
  nano23aa1n02x4               g161(.a(new_n256), .b(new_n220), .c(new_n223), .d(new_n233), .out0(new_n257));
  inv000aa1d42x5               g162(.a(\a[22] ), .o1(new_n258));
  xroi22aa1d04x5               g163(.a(new_n249), .b(\b[20] ), .c(new_n258), .d(\b[21] ), .out0(new_n259));
  oaoi03aa1n02x5               g164(.a(\a[22] ), .b(\b[21] ), .c(new_n250), .o1(new_n260));
  tech160nm_fiao0012aa1n02p5x5 g165(.a(new_n260), .b(new_n242), .c(new_n259), .o(new_n261));
  tech160nm_fixorc02aa1n03p5x5 g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n261), .c(new_n206), .d(new_n257), .o1(new_n263));
  aoi112aa1n02x5               g168(.a(new_n262), .b(new_n260), .c(new_n242), .d(new_n259), .o1(new_n264));
  aobi12aa1n02x5               g169(.a(new_n264), .b(new_n206), .c(new_n257), .out0(new_n265));
  norb02aa1n02x5               g170(.a(new_n263), .b(new_n265), .out0(\s[23] ));
  nor002aa1n03x5               g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fixorc02aa1n02p5x5 g173(.a(\a[24] ), .b(\b[23] ), .out0(new_n269));
  inv000aa1d42x5               g174(.a(\a[24] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\b[23] ), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n267), .b(new_n270), .c(new_n271), .o1(new_n272));
  oai112aa1n03x5               g177(.a(new_n263), .b(new_n272), .c(new_n271), .d(new_n270), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n269), .c(new_n263), .d(new_n268), .o1(\s[24] ));
  nano22aa1n02x5               g179(.a(new_n256), .b(new_n262), .c(new_n269), .out0(new_n275));
  and002aa1n02x5               g180(.a(new_n275), .b(new_n237), .o(new_n276));
  nanp02aa1n02x5               g181(.a(new_n206), .b(new_n276), .o1(new_n277));
  oaoi03aa1n02x5               g182(.a(new_n270), .b(new_n271), .c(new_n267), .o1(new_n278));
  and002aa1n02x5               g183(.a(new_n269), .b(new_n262), .o(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n260), .c(new_n242), .d(new_n259), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n280), .b(new_n278), .o1(new_n281));
  xorc02aa1n12x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n206), .d(new_n276), .o1(new_n283));
  nano22aa1n02x4               g188(.a(new_n282), .b(new_n280), .c(new_n278), .out0(new_n284));
  aobi12aa1n02x5               g189(.a(new_n283), .b(new_n284), .c(new_n277), .out0(\s[25] ));
  norp02aa1n02x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  tech160nm_fixorc02aa1n03p5x5 g192(.a(\a[26] ), .b(\b[25] ), .out0(new_n288));
  nanp02aa1n02x5               g193(.a(\b[25] ), .b(\a[26] ), .o1(new_n289));
  oai022aa1n02x5               g194(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n290));
  norb02aa1n02x5               g195(.a(new_n289), .b(new_n290), .out0(new_n291));
  tech160nm_finand02aa1n03p5x5 g196(.a(new_n283), .b(new_n291), .o1(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n283), .d(new_n287), .o1(\s[26] ));
  and002aa1n24x5               g198(.a(new_n288), .b(new_n282), .o(new_n294));
  nand23aa1n03x5               g199(.a(new_n275), .b(new_n237), .c(new_n294), .o1(new_n295));
  nanb02aa1n02x5               g200(.a(new_n295), .b(new_n206), .out0(new_n296));
  oaoi13aa1n09x5               g201(.a(new_n295), .b(new_n205), .c(new_n195), .d(new_n198), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n294), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n290), .b(new_n289), .o1(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n298), .c(new_n280), .d(new_n278), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  oai012aa1n06x5               g206(.a(new_n301), .b(new_n300), .c(new_n297), .o1(new_n302));
  aoi122aa1n02x5               g207(.a(new_n301), .b(new_n289), .c(new_n290), .d(new_n281), .e(new_n294), .o1(new_n303));
  aobi12aa1n03x7               g208(.a(new_n302), .b(new_n303), .c(new_n296), .out0(\s[27] ));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .out0(new_n307));
  oai022aa1n02x5               g212(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n308));
  aoi012aa1n02x5               g213(.a(new_n308), .b(\a[28] ), .c(\b[27] ), .o1(new_n309));
  tech160nm_finand02aa1n03p5x5 g214(.a(new_n302), .b(new_n309), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n307), .c(new_n302), .d(new_n306), .o1(\s[28] ));
  xorc02aa1n12x5               g216(.a(\a[29] ), .b(\b[28] ), .out0(new_n312));
  and002aa1n02x5               g217(.a(new_n307), .b(new_n301), .o(new_n313));
  oaih12aa1n02x5               g218(.a(new_n313), .b(new_n300), .c(new_n297), .o1(new_n314));
  aob012aa1n09x5               g219(.a(new_n308), .b(\b[27] ), .c(\a[28] ), .out0(new_n315));
  inv000aa1d42x5               g220(.a(new_n315), .o1(new_n316));
  inv000aa1d42x5               g221(.a(new_n312), .o1(new_n317));
  nona22aa1n02x4               g222(.a(new_n314), .b(new_n316), .c(new_n317), .out0(new_n318));
  oaoi13aa1n03x5               g223(.a(new_n316), .b(new_n313), .c(new_n300), .d(new_n297), .o1(new_n319));
  oaih12aa1n02x5               g224(.a(new_n318), .b(new_n319), .c(new_n312), .o1(\s[29] ));
  nanp02aa1n02x5               g225(.a(\b[0] ), .b(\a[1] ), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g227(.a(new_n317), .b(new_n301), .c(new_n307), .out0(new_n323));
  oaoi03aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .o1(new_n324));
  oaoi13aa1n03x5               g229(.a(new_n324), .b(new_n323), .c(new_n300), .d(new_n297), .o1(new_n325));
  norp02aa1n02x5               g230(.a(\b[29] ), .b(\a[30] ), .o1(new_n326));
  nanp02aa1n02x5               g231(.a(\b[29] ), .b(\a[30] ), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  oaih12aa1n02x5               g233(.a(new_n323), .b(new_n300), .c(new_n297), .o1(new_n329));
  and002aa1n02x5               g234(.a(\b[29] ), .b(\a[30] ), .o(new_n330));
  nona32aa1n02x5               g235(.a(new_n329), .b(new_n330), .c(new_n326), .d(new_n324), .out0(new_n331));
  oai012aa1n03x5               g236(.a(new_n331), .b(new_n325), .c(new_n328), .o1(\s[30] ));
  nano32aa1n02x4               g237(.a(new_n317), .b(new_n307), .c(new_n301), .d(new_n328), .out0(new_n333));
  oaih12aa1n02x5               g238(.a(new_n333), .b(new_n300), .c(new_n297), .o1(new_n334));
  oai012aa1n02x5               g239(.a(new_n315), .b(\b[28] ), .c(\a[29] ), .o1(new_n335));
  aoi112aa1n02x5               g240(.a(new_n330), .b(new_n326), .c(\a[29] ), .d(\b[28] ), .o1(new_n336));
  oai022aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(\b[30] ), .d(\a[31] ), .o1(new_n337));
  aoi122aa1n02x5               g242(.a(new_n337), .b(\b[30] ), .c(\a[31] ), .d(new_n335), .e(new_n336), .o1(new_n338));
  nand42aa1n02x5               g243(.a(new_n334), .b(new_n338), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n326), .b(new_n335), .c(new_n336), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[31] ), .b(\b[30] ), .out0(new_n341));
  aoai13aa1n03x5               g246(.a(new_n339), .b(new_n341), .c(new_n334), .d(new_n340), .o1(\s[31] ));
  inv000aa1n03x5               g247(.a(new_n103), .o1(new_n343));
  nanp02aa1n02x5               g248(.a(\b[1] ), .b(\a[2] ), .o1(new_n344));
  nano32aa1n02x4               g249(.a(new_n343), .b(new_n101), .c(new_n99), .d(new_n344), .out0(new_n345));
  aoi022aa1n02x5               g250(.a(new_n103), .b(new_n344), .c(new_n101), .d(new_n99), .o1(new_n346));
  norp02aa1n02x5               g251(.a(new_n345), .b(new_n346), .o1(\s[3] ));
  inv000aa1n03x5               g252(.a(new_n345), .o1(new_n348));
  xorc02aa1n02x5               g253(.a(\a[4] ), .b(\b[3] ), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n348), .c(new_n101), .out0(\s[4] ));
  nanb03aa1n02x5               g255(.a(new_n102), .b(new_n105), .c(new_n103), .out0(new_n351));
  norb02aa1n02x5               g256(.a(new_n112), .b(new_n111), .out0(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n352), .b(new_n351), .c(new_n107), .out0(\s[5] ));
  nanb02aa1n02x5               g258(.a(new_n110), .b(new_n109), .out0(new_n354));
  aoai13aa1n02x5               g259(.a(new_n354), .b(new_n111), .c(new_n108), .d(new_n112), .o1(new_n355));
  nona22aa1n02x4               g260(.a(new_n109), .b(new_n110), .c(new_n111), .out0(new_n356));
  aoai13aa1n02x5               g261(.a(new_n355), .b(new_n356), .c(new_n352), .d(new_n108), .o1(\s[6] ));
  nanb02aa1n02x5               g262(.a(new_n116), .b(new_n117), .out0(new_n358));
  oai012aa1n02x5               g263(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n359));
  nanb02aa1n02x5               g264(.a(new_n113), .b(new_n108), .out0(new_n360));
  xobna2aa1n03x5               g265(.a(new_n358), .b(new_n360), .c(new_n359), .out0(\s[7] ));
  tech160nm_fiao0012aa1n02p5x5 g266(.a(new_n358), .b(new_n360), .c(new_n359), .o(new_n362));
  xnbna2aa1n03x5               g267(.a(new_n122), .b(new_n362), .c(new_n126), .out0(\s[8] ));
  inv000aa1d42x5               g268(.a(new_n153), .o1(new_n364));
  nano22aa1n02x4               g269(.a(new_n124), .b(new_n128), .c(new_n153), .out0(new_n365));
  aboi22aa1n03x5               g270(.a(new_n195), .b(new_n364), .c(new_n365), .d(new_n120), .out0(\s[9] ));
endmodule


