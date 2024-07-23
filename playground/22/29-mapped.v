// Benchmark "adder" written by ABC on Wed Jul 17 23:29:26 2024

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
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n320, new_n322, new_n323, new_n324,
    new_n326, new_n327, new_n328, new_n330, new_n331, new_n333, new_n334,
    new_n335, new_n337, new_n338, new_n339, new_n340;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .o(new_n97));
  tech160nm_finand02aa1n03p5x5 g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[4] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[3] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n03p5x5 g006(.a(new_n101), .b(new_n99), .c(new_n100), .o1(new_n102));
  nand42aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norb02aa1n06x4               g008(.a(new_n103), .b(new_n101), .out0(new_n104));
  nanp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand22aa1n04x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nor042aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  oai112aa1n06x5               g012(.a(new_n104), .b(new_n105), .c(new_n106), .d(new_n107), .o1(new_n108));
  nanp02aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  oai122aa1n03x5               g014(.a(new_n109), .b(\a[7] ), .c(\b[6] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n110));
  aoi022aa1n02x7               g015(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n111));
  aoi022aa1n02x7               g016(.a(\b[7] ), .b(\a[8] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n112));
  oai022aa1n02x5               g017(.a(\a[6] ), .b(\b[5] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n111), .b(new_n112), .c(new_n110), .d(new_n113), .out0(new_n114));
  oai122aa1n02x7               g019(.a(new_n109), .b(\a[8] ), .c(\b[7] ), .d(\a[7] ), .e(\b[6] ), .o1(new_n115));
  nor002aa1n16x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nor002aa1n12x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  aoi022aa1n06x5               g022(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n118));
  tech160nm_fioai012aa1n04x5   g023(.a(new_n118), .b(new_n117), .c(new_n116), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[7] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[6] ), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n121), .b(new_n120), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .c(new_n122), .o1(new_n123));
  oab012aa1n06x5               g028(.a(new_n123), .b(new_n119), .c(new_n115), .out0(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n114), .c(new_n102), .d(new_n108), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  nanp02aa1n03x5               g031(.a(new_n125), .b(new_n126), .o1(new_n127));
  oa0012aa1n02x5               g032(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .o(new_n128));
  oai112aa1n06x5               g033(.a(new_n97), .b(new_n98), .c(\b[8] ), .d(\a[9] ), .o1(new_n129));
  nanb02aa1n06x5               g034(.a(new_n129), .b(new_n127), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n128), .c(new_n97), .d(new_n98), .o1(\s[10] ));
  nor022aa1n16x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi022aa1n02x5               g039(.a(new_n130), .b(new_n98), .c(new_n134), .d(new_n133), .o1(new_n135));
  aoi022aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n136));
  aoi013aa1n02x4               g041(.a(new_n135), .b(new_n133), .c(new_n130), .d(new_n136), .o1(\s[11] ));
  nand43aa1n02x5               g042(.a(new_n130), .b(new_n133), .c(new_n136), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  tech160nm_finand02aa1n03p5x5 g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  norp02aa1n02x5               g046(.a(new_n139), .b(new_n132), .o1(new_n142));
  nanp03aa1n02x5               g047(.a(new_n138), .b(new_n140), .c(new_n142), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n141), .c(new_n133), .d(new_n138), .o1(\s[12] ));
  nanp02aa1n02x5               g049(.a(new_n97), .b(new_n98), .o1(new_n145));
  nanb02aa1n02x5               g050(.a(new_n132), .b(new_n134), .out0(new_n146));
  nona23aa1n09x5               g051(.a(new_n141), .b(new_n126), .c(new_n145), .d(new_n146), .out0(new_n147));
  nanb02aa1n06x5               g052(.a(new_n147), .b(new_n125), .out0(new_n148));
  aobi12aa1n06x5               g053(.a(new_n142), .b(new_n129), .c(new_n136), .out0(new_n149));
  nanb02aa1n02x5               g054(.a(new_n149), .b(new_n140), .out0(new_n150));
  xorc02aa1n02x5               g055(.a(\a[13] ), .b(\b[12] ), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n148), .c(new_n150), .out0(\s[13] ));
  nand22aa1n03x5               g057(.a(new_n148), .b(new_n150), .o1(new_n153));
  nor042aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nor042aa1n04x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand02aa1n12x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n154), .c(new_n153), .d(new_n151), .o1(new_n158));
  nona22aa1n02x4               g063(.a(new_n156), .b(new_n155), .c(new_n154), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n158), .b(new_n159), .c(new_n151), .d(new_n153), .o1(\s[14] ));
  inv000aa1d42x5               g065(.a(\a[13] ), .o1(new_n161));
  inv020aa1n04x5               g066(.a(\a[14] ), .o1(new_n162));
  xroi22aa1d06x4               g067(.a(new_n161), .b(\b[12] ), .c(new_n162), .d(\b[13] ), .out0(new_n163));
  nanp02aa1n03x5               g068(.a(new_n153), .b(new_n163), .o1(new_n164));
  tech160nm_fioai012aa1n05x5   g069(.a(new_n156), .b(new_n155), .c(new_n154), .o1(new_n165));
  xnrc02aa1n12x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  xobna2aa1n03x5               g071(.a(new_n166), .b(new_n164), .c(new_n165), .out0(\s[15] ));
  inv000aa1d42x5               g072(.a(\a[15] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(\b[14] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n163), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n165), .b(new_n170), .c(new_n148), .d(new_n150), .o1(new_n171));
  oaoi03aa1n02x5               g076(.a(new_n168), .b(new_n169), .c(new_n171), .o1(new_n172));
  nor022aa1n06x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  inv000aa1n02x5               g078(.a(new_n173), .o1(new_n174));
  tech160nm_finand02aa1n03p5x5 g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  inv020aa1n02x5               g080(.a(new_n175), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n176), .b(new_n173), .c(new_n168), .d(new_n169), .o1(new_n177));
  aoai13aa1n02x7               g082(.a(new_n177), .b(new_n166), .c(new_n164), .d(new_n165), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n178), .b(new_n172), .c(new_n175), .d(new_n174), .o1(\s[16] ));
  nano22aa1n02x4               g084(.a(new_n166), .b(new_n174), .c(new_n175), .out0(new_n180));
  nano22aa1d15x5               g085(.a(new_n147), .b(new_n163), .c(new_n180), .out0(new_n181));
  nanp02aa1n06x5               g086(.a(new_n125), .b(new_n181), .o1(new_n182));
  oaoi03aa1n03x5               g087(.a(\a[15] ), .b(\b[14] ), .c(new_n165), .o1(new_n183));
  nand42aa1n02x5               g088(.a(new_n183), .b(new_n175), .o1(new_n184));
  oai022aa1n02x5               g089(.a(new_n168), .b(new_n169), .c(\b[15] ), .d(\a[16] ), .o1(new_n185));
  oai012aa1n02x5               g090(.a(new_n140), .b(\b[12] ), .c(\a[13] ), .o1(new_n186));
  oai012aa1n02x5               g091(.a(new_n156), .b(\b[14] ), .c(\a[15] ), .o1(new_n187));
  aoi112aa1n02x5               g092(.a(new_n187), .b(new_n155), .c(\a[13] ), .d(\b[12] ), .o1(new_n188));
  nona32aa1n06x5               g093(.a(new_n188), .b(new_n185), .c(new_n186), .d(new_n176), .out0(new_n189));
  norp02aa1n02x5               g094(.a(new_n149), .b(new_n189), .o1(new_n190));
  nano22aa1n03x7               g095(.a(new_n190), .b(new_n184), .c(new_n174), .out0(new_n191));
  xorc02aa1n12x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n182), .c(new_n191), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  nanb02aa1n12x5               g099(.a(\b[16] ), .b(new_n194), .out0(new_n195));
  oai112aa1n06x5               g100(.a(new_n184), .b(new_n174), .c(new_n149), .d(new_n189), .o1(new_n196));
  aoai13aa1n03x5               g101(.a(new_n192), .b(new_n196), .c(new_n125), .d(new_n181), .o1(new_n197));
  tech160nm_fixnrc02aa1n04x5   g102(.a(\b[17] ), .b(\a[18] ), .out0(new_n198));
  xobna2aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n195), .out0(\s[18] ));
  norb02aa1n02x5               g104(.a(new_n192), .b(new_n198), .out0(new_n200));
  aoai13aa1n06x5               g105(.a(new_n200), .b(new_n196), .c(new_n125), .d(new_n181), .o1(new_n201));
  oaoi03aa1n09x5               g106(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  nor042aa1n09x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand42aa1n08x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norb02aa1n06x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n201), .c(new_n203), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n06x5               g113(.a(new_n206), .b(new_n201), .c(new_n203), .out0(new_n209));
  nor002aa1n10x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nand42aa1n20x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  oabi12aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n204), .out0(new_n213));
  norb03aa1n02x5               g118(.a(new_n211), .b(new_n204), .c(new_n210), .out0(new_n214));
  oaib12aa1n03x5               g119(.a(new_n213), .b(new_n209), .c(new_n214), .out0(\s[20] ));
  nano23aa1n06x5               g120(.a(new_n204), .b(new_n210), .c(new_n211), .d(new_n205), .out0(new_n216));
  nanb03aa1n09x5               g121(.a(new_n198), .b(new_n216), .c(new_n192), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n196), .c(new_n125), .d(new_n181), .o1(new_n219));
  norp02aa1n02x5               g124(.a(\b[17] ), .b(\a[18] ), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n221));
  oai112aa1n03x5               g126(.a(new_n206), .b(new_n212), .c(new_n221), .d(new_n220), .o1(new_n222));
  oaih12aa1n06x5               g127(.a(new_n211), .b(new_n210), .c(new_n204), .o1(new_n223));
  nand02aa1n03x5               g128(.a(new_n222), .b(new_n223), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n219), .c(new_n225), .out0(\s[21] ));
  norp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  nand42aa1n06x5               g134(.a(new_n182), .b(new_n191), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n226), .b(new_n224), .c(new_n230), .d(new_n218), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[22] ), .b(\b[21] ), .out0(new_n232));
  xnrc02aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .out0(new_n233));
  nanp02aa1n02x5               g138(.a(\b[21] ), .b(\a[22] ), .o1(new_n234));
  oai022aa1n02x5               g139(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  aoai13aa1n02x7               g141(.a(new_n236), .b(new_n233), .c(new_n219), .d(new_n225), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n231), .d(new_n229), .o1(\s[22] ));
  nanp02aa1n02x5               g143(.a(new_n232), .b(new_n226), .o1(new_n239));
  nano23aa1n02x4               g144(.a(new_n239), .b(new_n198), .c(new_n216), .d(new_n192), .out0(new_n240));
  nanp02aa1n02x5               g145(.a(new_n235), .b(new_n234), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n239), .c(new_n222), .d(new_n223), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n242), .c(new_n230), .d(new_n240), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n242), .c(new_n230), .d(new_n240), .o1(new_n245));
  norb02aa1n02x7               g150(.a(new_n244), .b(new_n245), .out0(\s[23] ));
  nor042aa1n03x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  tech160nm_fixorc02aa1n05x5   g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  oai022aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(\a[24] ), .c(\b[23] ), .o1(new_n251));
  tech160nm_finand02aa1n03p5x5 g156(.a(new_n244), .b(new_n251), .o1(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n249), .c(new_n248), .d(new_n244), .o1(\s[24] ));
  orn002aa1n02x5               g158(.a(\a[22] ), .b(\b[21] ), .o(new_n254));
  nano22aa1n02x4               g159(.a(new_n233), .b(new_n254), .c(new_n234), .out0(new_n255));
  nano32aa1n02x5               g160(.a(new_n217), .b(new_n249), .c(new_n255), .d(new_n243), .out0(new_n256));
  inv000aa1n02x5               g161(.a(new_n223), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n255), .b(new_n257), .c(new_n216), .d(new_n202), .o1(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .out0(new_n259));
  norb02aa1n09x5               g164(.a(new_n249), .b(new_n259), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  oaoi03aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .o1(new_n262));
  inv040aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n261), .c(new_n258), .d(new_n241), .o1(new_n264));
  tech160nm_fixorc02aa1n03p5x5 g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n264), .c(new_n230), .d(new_n256), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(new_n265), .b(new_n264), .c(new_n230), .d(new_n256), .o1(new_n267));
  norb02aa1n03x4               g172(.a(new_n266), .b(new_n267), .out0(\s[25] ));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .out0(new_n271));
  nanp02aa1n02x5               g176(.a(\b[25] ), .b(\a[26] ), .o1(new_n272));
  oai022aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n273));
  norb02aa1n02x5               g178(.a(new_n272), .b(new_n273), .out0(new_n274));
  nand42aa1n03x5               g179(.a(new_n266), .b(new_n274), .o1(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n271), .c(new_n270), .d(new_n266), .o1(\s[26] ));
  and002aa1n02x5               g181(.a(new_n271), .b(new_n265), .o(new_n277));
  nano32aa1n03x7               g182(.a(new_n217), .b(new_n277), .c(new_n255), .d(new_n260), .out0(new_n278));
  aoai13aa1n12x5               g183(.a(new_n278), .b(new_n196), .c(new_n125), .d(new_n181), .o1(new_n279));
  aoai13aa1n02x7               g184(.a(new_n277), .b(new_n262), .c(new_n242), .d(new_n260), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(new_n273), .b(new_n272), .o1(new_n281));
  nand43aa1n03x5               g186(.a(new_n279), .b(new_n280), .c(new_n281), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  aoi122aa1n02x5               g188(.a(new_n283), .b(new_n272), .c(new_n273), .d(new_n264), .e(new_n277), .o1(new_n284));
  aoi022aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n282), .d(new_n283), .o1(\s[27] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  xnrc02aa1n12x5               g191(.a(\b[27] ), .b(\a[28] ), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n282), .d(new_n283), .o1(new_n288));
  aoi022aa1n06x5               g193(.a(new_n264), .b(new_n277), .c(new_n272), .d(new_n273), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n283), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n287), .b(new_n286), .o1(new_n291));
  aoai13aa1n02x7               g196(.a(new_n291), .b(new_n290), .c(new_n289), .d(new_n279), .o1(new_n292));
  nanp02aa1n03x5               g197(.a(new_n288), .b(new_n292), .o1(\s[28] ));
  norb02aa1n03x5               g198(.a(new_n283), .b(new_n287), .out0(new_n294));
  nand02aa1n02x5               g199(.a(new_n282), .b(new_n294), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n294), .o1(new_n296));
  tech160nm_fiao0012aa1n02p5x5 g201(.a(new_n291), .b(\a[28] ), .c(\b[27] ), .o(new_n297));
  aoai13aa1n02x7               g202(.a(new_n297), .b(new_n296), .c(new_n289), .d(new_n279), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .out0(new_n299));
  norb02aa1n02x5               g204(.a(new_n297), .b(new_n299), .out0(new_n300));
  aoi022aa1n03x5               g205(.a(new_n298), .b(new_n299), .c(new_n295), .d(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g207(.a(new_n287), .b(new_n299), .c(new_n283), .out0(new_n303));
  nanb02aa1n03x5               g208(.a(new_n303), .b(new_n282), .out0(new_n304));
  oaoi03aa1n09x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n303), .c(new_n289), .d(new_n279), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n305), .b(new_n308), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n304), .d(new_n309), .o1(\s[30] ));
  nand03aa1n02x5               g215(.a(new_n294), .b(new_n299), .c(new_n308), .o1(new_n311));
  nanb02aa1n03x5               g216(.a(new_n311), .b(new_n282), .out0(new_n312));
  xorc02aa1n02x5               g217(.a(\a[31] ), .b(\b[30] ), .out0(new_n313));
  inv000aa1d42x5               g218(.a(\a[30] ), .o1(new_n314));
  inv000aa1d42x5               g219(.a(\b[29] ), .o1(new_n315));
  tech160nm_fioaoi03aa1n03p5x5 g220(.a(new_n314), .b(new_n315), .c(new_n305), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n316), .b(new_n313), .out0(new_n317));
  aoai13aa1n02x7               g222(.a(new_n316), .b(new_n311), .c(new_n289), .d(new_n279), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n318), .b(new_n313), .c(new_n312), .d(new_n317), .o1(\s[31] ));
  oai012aa1n02x5               g224(.a(new_n105), .b(new_n107), .c(new_n106), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaib12aa1n06x5               g226(.a(new_n102), .b(new_n320), .c(new_n104), .out0(new_n322));
  xorc02aa1n02x5               g227(.a(\a[4] ), .b(\b[3] ), .out0(new_n323));
  norp02aa1n02x5               g228(.a(new_n323), .b(new_n101), .o1(new_n324));
  aoi022aa1n02x5               g229(.a(new_n322), .b(new_n323), .c(new_n108), .d(new_n324), .o1(\s[4] ));
  xorc02aa1n02x5               g230(.a(\a[5] ), .b(\b[4] ), .out0(new_n326));
  oaoi13aa1n02x5               g231(.a(new_n326), .b(new_n322), .c(new_n99), .d(new_n100), .o1(new_n327));
  oai112aa1n02x5               g232(.a(new_n322), .b(new_n326), .c(new_n100), .d(new_n99), .o1(new_n328));
  norb02aa1n02x5               g233(.a(new_n328), .b(new_n327), .out0(\s[5] ));
  inv000aa1d42x5               g234(.a(new_n116), .o1(new_n330));
  norb02aa1n02x5               g235(.a(new_n109), .b(new_n117), .out0(new_n331));
  xnbna2aa1n03x5               g236(.a(new_n331), .b(new_n328), .c(new_n330), .out0(\s[6] ));
  inv000aa1d42x5               g237(.a(new_n117), .o1(new_n333));
  aob012aa1n06x5               g238(.a(new_n331), .b(new_n328), .c(new_n330), .out0(new_n334));
  xorc02aa1n02x5               g239(.a(\a[7] ), .b(\b[6] ), .out0(new_n335));
  xnbna2aa1n03x5               g240(.a(new_n335), .b(new_n334), .c(new_n333), .out0(\s[7] ));
  aobi12aa1n06x5               g241(.a(new_n335), .b(new_n334), .c(new_n333), .out0(new_n337));
  xnrc02aa1n02x5               g242(.a(\b[7] ), .b(\a[8] ), .out0(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n337), .c(new_n120), .d(new_n121), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n122), .b(new_n338), .out0(new_n340));
  oaib12aa1n02x5               g245(.a(new_n339), .b(new_n337), .c(new_n340), .out0(\s[8] ));
  xorb03aa1n02x5               g246(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


