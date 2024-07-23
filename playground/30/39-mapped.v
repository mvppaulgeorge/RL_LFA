// Benchmark "adder" written by ABC on Thu Jul 18 03:41:25 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n313, new_n314, new_n316, new_n317, new_n319,
    new_n321, new_n323, new_n324, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv020aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n12x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  inv000aa1d42x5               g003(.a(\a[3] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[4] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[2] ), .o1(new_n101));
  aboi22aa1n09x5               g006(.a(\b[3] ), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n102));
  xnrc02aa1n12x5               g007(.a(\b[2] ), .b(\a[3] ), .out0(new_n103));
  tech160nm_finand02aa1n05x5   g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nand02aa1d28x5               g009(.a(\b[0] ), .b(\a[1] ), .o1(new_n105));
  nor042aa1d18x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  oai012aa1n18x5               g011(.a(new_n104), .b(new_n106), .c(new_n105), .o1(new_n107));
  oai012aa1n12x5               g012(.a(new_n102), .b(new_n103), .c(new_n107), .o1(new_n108));
  inv040aa1d32x5               g013(.a(\a[6] ), .o1(new_n109));
  inv030aa1d32x5               g014(.a(\b[5] ), .o1(new_n110));
  nand42aa1n02x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nanp02aa1n09x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  oai112aa1n03x5               g017(.a(new_n111), .b(new_n112), .c(\b[7] ), .d(\a[8] ), .o1(new_n113));
  inv040aa1d30x5               g018(.a(\a[5] ), .o1(new_n114));
  aoi022aa1d24x5               g019(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n115));
  oaib12aa1n18x5               g020(.a(new_n115), .b(\b[4] ), .c(new_n114), .out0(new_n116));
  nanp02aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nor022aa1n12x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand42aa1d28x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nanb03aa1n03x5               g024(.a(new_n118), .b(new_n119), .c(new_n117), .out0(new_n120));
  nor043aa1n06x5               g025(.a(new_n113), .b(new_n120), .c(new_n116), .o1(new_n121));
  oai022aa1n02x5               g026(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n117), .o1(new_n123));
  oai112aa1n03x5               g028(.a(new_n112), .b(new_n119), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  oai012aa1n02x7               g029(.a(new_n117), .b(\b[6] ), .c(\a[7] ), .o1(new_n125));
  aboi22aa1n02x7               g030(.a(\b[4] ), .b(new_n114), .c(new_n110), .d(new_n109), .out0(new_n126));
  oai013aa1n03x5               g031(.a(new_n123), .b(new_n124), .c(new_n126), .d(new_n125), .o1(new_n127));
  tech160nm_fixorc02aa1n03p5x5 g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n127), .c(new_n121), .d(new_n108), .o1(new_n129));
  tech160nm_fixnrc02aa1n04x5   g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  xobna2aa1n03x5               g035(.a(new_n130), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  tech160nm_fioaoi03aa1n03p5x5 g036(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n132));
  inv000aa1n02x5               g037(.a(new_n132), .o1(new_n133));
  aoai13aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n129), .d(new_n98), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1d24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n138));
  nor002aa1d32x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1d16x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n139), .b(new_n140), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  aoi112aa1n02x5               g047(.a(new_n136), .b(new_n142), .c(new_n134), .d(new_n137), .o1(new_n143));
  oab012aa1n02x5               g048(.a(new_n143), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  aoi012aa1n12x5               g049(.a(new_n127), .b(new_n121), .c(new_n108), .o1(new_n145));
  inv000aa1n06x5               g050(.a(new_n145), .o1(new_n146));
  nona23aa1d18x5               g051(.a(new_n140), .b(new_n137), .c(new_n136), .d(new_n139), .out0(new_n147));
  norb03aa1n12x5               g052(.a(new_n128), .b(new_n147), .c(new_n130), .out0(new_n148));
  ao0012aa1n03x7               g053(.a(new_n139), .b(new_n136), .c(new_n140), .o(new_n149));
  oabi12aa1n02x5               g054(.a(new_n149), .b(new_n133), .c(new_n147), .out0(new_n150));
  nor042aa1d18x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  tech160nm_finand02aa1n03p5x5 g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  aoai13aa1n03x5               g058(.a(new_n153), .b(new_n150), .c(new_n146), .d(new_n148), .o1(new_n154));
  aoi112aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n146), .d(new_n148), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(\s[13] ));
  inv040aa1n02x5               g061(.a(new_n151), .o1(new_n157));
  xorc02aa1n02x5               g062(.a(\a[14] ), .b(\b[13] ), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n154), .c(new_n157), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n04x5   g064(.a(\b[13] ), .b(\a[14] ), .out0(new_n160));
  nano22aa1n06x5               g065(.a(new_n160), .b(new_n157), .c(new_n152), .out0(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n150), .c(new_n146), .d(new_n148), .o1(new_n162));
  oao003aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .carry(new_n163));
  xorc02aa1n12x5               g068(.a(\a[15] ), .b(\b[14] ), .out0(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n164), .b(new_n162), .c(new_n163), .out0(\s[15] ));
  aob012aa1n03x5               g070(.a(new_n164), .b(new_n162), .c(new_n163), .out0(new_n166));
  nor042aa1n09x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n164), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n169), .c(new_n162), .d(new_n163), .o1(new_n170));
  tech160nm_fixorc02aa1n04x5   g075(.a(\a[16] ), .b(\b[15] ), .out0(new_n171));
  norp02aa1n02x5               g076(.a(new_n171), .b(new_n167), .o1(new_n172));
  aoi022aa1n03x5               g077(.a(new_n170), .b(new_n171), .c(new_n166), .d(new_n172), .o1(\s[16] ));
  nanp02aa1n02x5               g078(.a(new_n158), .b(new_n153), .o1(new_n174));
  nand22aa1n09x5               g079(.a(new_n171), .b(new_n164), .o1(new_n175));
  nona22aa1n09x5               g080(.a(new_n148), .b(new_n174), .c(new_n175), .out0(new_n176));
  nano23aa1n06x5               g081(.a(new_n136), .b(new_n139), .c(new_n140), .d(new_n137), .out0(new_n177));
  aoai13aa1n04x5               g082(.a(new_n161), .b(new_n149), .c(new_n177), .d(new_n132), .o1(new_n178));
  tech160nm_fiao0012aa1n05x5   g083(.a(new_n175), .b(new_n178), .c(new_n163), .o(new_n179));
  oao003aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n168), .carry(new_n180));
  oai112aa1n06x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .d(new_n145), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv020aa1n12x5               g087(.a(\a[17] ), .o1(new_n183));
  nanb02aa1n12x5               g088(.a(\b[16] ), .b(new_n183), .out0(new_n184));
  nor042aa1n06x5               g089(.a(new_n176), .b(new_n145), .o1(new_n185));
  aoai13aa1n09x5               g090(.a(new_n180), .b(new_n175), .c(new_n178), .d(new_n163), .o1(new_n186));
  xorc02aa1n02x5               g091(.a(\a[17] ), .b(\b[16] ), .out0(new_n187));
  oaih12aa1n02x5               g092(.a(new_n187), .b(new_n186), .c(new_n185), .o1(new_n188));
  tech160nm_fixorc02aa1n03p5x5 g093(.a(\a[18] ), .b(\b[17] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n188), .c(new_n184), .out0(\s[18] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  xroi22aa1d04x5               g096(.a(new_n183), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n192));
  oai012aa1n06x5               g097(.a(new_n192), .b(new_n186), .c(new_n185), .o1(new_n193));
  oaih22aa1d12x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n18x5               g099(.a(new_n194), .b(new_n191), .c(\b[17] ), .out0(new_n195));
  nor002aa1d32x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand22aa1n12x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n06x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n193), .c(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n03x5               g105(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n201));
  aoai13aa1n03x5               g106(.a(new_n198), .b(new_n201), .c(new_n181), .d(new_n192), .o1(new_n202));
  inv040aa1n09x5               g107(.a(new_n196), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n198), .o1(new_n204));
  aoai13aa1n03x5               g109(.a(new_n203), .b(new_n204), .c(new_n193), .d(new_n195), .o1(new_n205));
  nor002aa1d32x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1d24x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aoib12aa1n02x5               g113(.a(new_n196), .b(new_n207), .c(new_n206), .out0(new_n209));
  aoi022aa1n03x5               g114(.a(new_n205), .b(new_n208), .c(new_n202), .d(new_n209), .o1(\s[20] ));
  nona23aa1d18x5               g115(.a(new_n207), .b(new_n197), .c(new_n196), .d(new_n206), .out0(new_n211));
  nano22aa1n03x7               g116(.a(new_n211), .b(new_n187), .c(new_n189), .out0(new_n212));
  oai012aa1n06x5               g117(.a(new_n212), .b(new_n186), .c(new_n185), .o1(new_n213));
  oaoi03aa1n12x5               g118(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .o1(new_n214));
  oabi12aa1n18x5               g119(.a(new_n214), .b(new_n211), .c(new_n195), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n213), .c(new_n216), .out0(\s[21] ));
  aoai13aa1n03x5               g123(.a(new_n217), .b(new_n215), .c(new_n181), .d(new_n212), .o1(new_n219));
  nor002aa1d32x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  inv030aa1n03x5               g125(.a(new_n220), .o1(new_n221));
  inv000aa1n02x5               g126(.a(new_n217), .o1(new_n222));
  aoai13aa1n02x7               g127(.a(new_n221), .b(new_n222), .c(new_n213), .d(new_n216), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  norp02aa1n02x5               g129(.a(new_n224), .b(new_n220), .o1(new_n225));
  aoi022aa1n03x5               g130(.a(new_n223), .b(new_n224), .c(new_n219), .d(new_n225), .o1(\s[22] ));
  nano23aa1n06x5               g131(.a(new_n196), .b(new_n206), .c(new_n207), .d(new_n197), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  xnrc02aa1n03x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  nano22aa1n06x5               g134(.a(new_n229), .b(new_n221), .c(new_n228), .out0(new_n230));
  and003aa1n02x5               g135(.a(new_n192), .b(new_n230), .c(new_n227), .o(new_n231));
  oai012aa1n06x5               g136(.a(new_n231), .b(new_n186), .c(new_n185), .o1(new_n232));
  oao003aa1n02x5               g137(.a(\a[22] ), .b(\b[21] ), .c(new_n221), .carry(new_n233));
  inv000aa1n02x5               g138(.a(new_n233), .o1(new_n234));
  aoi012aa1n02x5               g139(.a(new_n234), .b(new_n215), .c(new_n230), .o1(new_n235));
  nanp02aa1n03x5               g140(.a(new_n232), .b(new_n235), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n237), .b(new_n234), .c(new_n215), .d(new_n230), .o1(new_n238));
  aoi022aa1n02x5               g143(.a(new_n236), .b(new_n237), .c(new_n232), .d(new_n238), .o1(\s[23] ));
  nanp02aa1n03x5               g144(.a(new_n236), .b(new_n237), .o1(new_n240));
  nor042aa1n09x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n237), .o1(new_n243));
  aoai13aa1n02x7               g148(.a(new_n242), .b(new_n243), .c(new_n232), .d(new_n235), .o1(new_n244));
  xorc02aa1n03x5               g149(.a(\a[24] ), .b(\b[23] ), .out0(new_n245));
  norp02aa1n02x5               g150(.a(new_n245), .b(new_n241), .o1(new_n246));
  aoi022aa1n03x5               g151(.a(new_n244), .b(new_n245), .c(new_n240), .d(new_n246), .o1(\s[24] ));
  and002aa1n12x5               g152(.a(new_n245), .b(new_n237), .o(new_n248));
  inv000aa1n06x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n02x5               g154(.a(new_n249), .b(new_n192), .c(new_n227), .d(new_n230), .out0(new_n250));
  oaih12aa1n04x5               g155(.a(new_n250), .b(new_n186), .c(new_n185), .o1(new_n251));
  aoai13aa1n04x5               g156(.a(new_n230), .b(new_n214), .c(new_n227), .d(new_n201), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .c(new_n242), .carry(new_n253));
  aoai13aa1n12x5               g158(.a(new_n253), .b(new_n249), .c(new_n252), .d(new_n233), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  xnbna2aa1n03x5               g161(.a(new_n256), .b(new_n251), .c(new_n255), .out0(\s[25] ));
  aoai13aa1n02x7               g162(.a(new_n256), .b(new_n254), .c(new_n181), .d(new_n250), .o1(new_n258));
  nor042aa1n03x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n256), .o1(new_n261));
  aoai13aa1n02x7               g166(.a(new_n260), .b(new_n261), .c(new_n251), .d(new_n255), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .out0(new_n263));
  norp02aa1n02x5               g168(.a(new_n263), .b(new_n259), .o1(new_n264));
  aoi022aa1n03x5               g169(.a(new_n262), .b(new_n263), .c(new_n258), .d(new_n264), .o1(\s[26] ));
  and002aa1n03x5               g170(.a(new_n263), .b(new_n256), .o(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  nano32aa1n09x5               g172(.a(new_n267), .b(new_n212), .c(new_n230), .d(new_n248), .out0(new_n268));
  oai012aa1n12x5               g173(.a(new_n268), .b(new_n186), .c(new_n185), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n270));
  aobi12aa1n12x5               g175(.a(new_n270), .b(new_n254), .c(new_n266), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  aoai13aa1n03x5               g178(.a(new_n248), .b(new_n234), .c(new_n215), .d(new_n230), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n270), .b(new_n267), .c(new_n274), .d(new_n253), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n272), .b(new_n275), .c(new_n181), .d(new_n268), .o1(new_n276));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv000aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n272), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n278), .b(new_n279), .c(new_n271), .d(new_n269), .o1(new_n280));
  tech160nm_fixorc02aa1n03p5x5 g185(.a(\a[28] ), .b(\b[27] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n277), .o1(new_n282));
  aoi022aa1n03x5               g187(.a(new_n280), .b(new_n281), .c(new_n276), .d(new_n282), .o1(\s[28] ));
  and002aa1n02x5               g188(.a(new_n281), .b(new_n272), .o(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n275), .c(new_n181), .d(new_n268), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n284), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n271), .d(new_n269), .o1(new_n288));
  tech160nm_fixorc02aa1n03p5x5 g193(.a(\a[29] ), .b(\b[28] ), .out0(new_n289));
  norb02aa1n02x5               g194(.a(new_n287), .b(new_n289), .out0(new_n290));
  aoi022aa1n02x7               g195(.a(new_n288), .b(new_n289), .c(new_n285), .d(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n105), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g197(.a(new_n279), .b(new_n281), .c(new_n289), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n275), .c(new_n181), .d(new_n268), .o1(new_n294));
  inv000aa1n02x5               g199(.a(new_n293), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n295), .c(new_n271), .d(new_n269), .o1(new_n297));
  tech160nm_fixorc02aa1n05x5   g202(.a(\a[30] ), .b(\b[29] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[30] ));
  xorc02aa1n02x5               g205(.a(\a[31] ), .b(\b[30] ), .out0(new_n301));
  nano32aa1n02x5               g206(.a(new_n279), .b(new_n298), .c(new_n281), .d(new_n289), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n275), .c(new_n181), .d(new_n268), .o1(new_n303));
  inv000aa1n02x5               g208(.a(new_n302), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n304), .c(new_n271), .d(new_n269), .o1(new_n306));
  and002aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .o(new_n307));
  oabi12aa1n02x5               g212(.a(new_n301), .b(\a[30] ), .c(\b[29] ), .out0(new_n308));
  oab012aa1n02x4               g213(.a(new_n308), .b(new_n296), .c(new_n307), .out0(new_n309));
  aoi022aa1n02x7               g214(.a(new_n306), .b(new_n301), .c(new_n303), .d(new_n309), .o1(\s[31] ));
  xorb03aa1n02x5               g215(.a(new_n107), .b(\b[2] ), .c(new_n99), .out0(\s[3] ));
  norp02aa1n02x5               g216(.a(new_n103), .b(new_n107), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[4] ), .b(\b[3] ), .out0(new_n313));
  aoi012aa1n02x5               g218(.a(new_n313), .b(new_n99), .c(new_n101), .o1(new_n314));
  aboi22aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n108), .d(new_n313), .out0(\s[4] ));
  xnrc02aa1n02x5               g220(.a(\b[4] ), .b(\a[5] ), .out0(new_n316));
  oaib12aa1n02x5               g221(.a(new_n108), .b(new_n100), .c(\b[3] ), .out0(new_n317));
  aboi22aa1n03x5               g222(.a(new_n116), .b(new_n108), .c(new_n317), .d(new_n316), .out0(\s[5] ));
  obai22aa1n03x5               g223(.a(new_n108), .b(new_n116), .c(\a[5] ), .d(\b[4] ), .out0(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g225(.a(new_n111), .b(new_n319), .c(new_n112), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  xorc02aa1n02x5               g227(.a(\a[8] ), .b(\b[7] ), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n118), .c(new_n321), .d(new_n119), .o1(new_n324));
  aoi112aa1n02x5               g229(.a(new_n323), .b(new_n118), .c(new_n321), .d(new_n119), .o1(new_n325));
  norb02aa1n03x4               g230(.a(new_n324), .b(new_n325), .out0(\s[8] ));
  xorb03aa1n02x5               g231(.a(new_n145), .b(\b[8] ), .c(new_n97), .out0(\s[9] ));
endmodule


