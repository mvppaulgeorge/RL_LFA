// Benchmark "adder" written by ABC on Wed Jul 17 23:19:00 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n320,
    new_n322, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1n04x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  tech160nm_finand02aa1n05x5   g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  norb02aa1n06x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  xorc02aa1n12x5               g008(.a(\a[7] ), .b(\b[6] ), .out0(new_n104));
  nor002aa1n04x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nand02aa1d04x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norp02aa1n04x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nanp02aa1n03x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nona23aa1n09x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .d(new_n107), .out0(new_n109));
  nanb03aa1n02x5               g014(.a(new_n109), .b(new_n103), .c(new_n104), .out0(new_n110));
  nor002aa1n12x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n06x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  nand02aa1d04x5               g021(.a(\b[0] ), .b(\a[1] ), .o1(new_n117));
  nor042aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  oai012aa1n12x5               g023(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n119));
  oa0012aa1n12x5               g024(.a(new_n112), .b(new_n113), .c(new_n111), .o(new_n120));
  oab012aa1n06x5               g025(.a(new_n120), .b(new_n115), .c(new_n119), .out0(new_n121));
  aoi112aa1n02x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  aoi112aa1n09x5               g027(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n123));
  oai112aa1n04x5               g028(.a(new_n104), .b(new_n103), .c(new_n105), .d(new_n123), .o1(new_n124));
  nona22aa1n12x5               g029(.a(new_n124), .b(new_n122), .c(new_n101), .out0(new_n125));
  oabi12aa1n06x5               g030(.a(new_n125), .b(new_n121), .c(new_n110), .out0(new_n126));
  nand42aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n99), .b(new_n100), .c(new_n126), .d(new_n127), .o1(new_n128));
  inv000aa1d42x5               g033(.a(new_n98), .o1(new_n129));
  nano22aa1n03x7               g034(.a(new_n109), .b(new_n104), .c(new_n103), .out0(new_n130));
  oabi12aa1n18x5               g035(.a(new_n120), .b(new_n115), .c(new_n119), .out0(new_n131));
  norb02aa1n02x5               g036(.a(new_n127), .b(new_n100), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n125), .c(new_n131), .d(new_n130), .o1(new_n133));
  nona32aa1n03x5               g038(.a(new_n133), .b(new_n100), .c(new_n129), .d(new_n97), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n128), .b(new_n134), .o1(\s[10] ));
  nor002aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n09x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n134), .c(new_n98), .out0(\s[11] ));
  aoi013aa1n06x4               g044(.a(new_n136), .b(new_n134), .c(new_n98), .d(new_n137), .o1(new_n140));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  inv000aa1d42x5               g046(.a(new_n141), .o1(new_n142));
  nand02aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n140), .b(new_n143), .c(new_n142), .out0(\s[12] ));
  norb02aa1n03x5               g049(.a(new_n137), .b(new_n136), .out0(new_n145));
  norb02aa1n06x5               g050(.a(new_n143), .b(new_n141), .out0(new_n146));
  nano32aa1n03x7               g051(.a(new_n99), .b(new_n146), .c(new_n132), .d(new_n145), .out0(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n125), .c(new_n131), .d(new_n130), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n136), .b(new_n143), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n141), .b(new_n143), .out0(new_n150));
  oai022aa1n02x5               g055(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n151));
  nano23aa1n06x5               g056(.a(new_n150), .b(new_n138), .c(new_n151), .d(new_n98), .out0(new_n152));
  nano22aa1n03x7               g057(.a(new_n152), .b(new_n142), .c(new_n149), .out0(new_n153));
  nor042aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand02aa1d16x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n155), .b(new_n154), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n148), .c(new_n153), .out0(\s[13] ));
  nand22aa1n03x5               g062(.a(new_n148), .b(new_n153), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n154), .b(new_n158), .c(new_n155), .o1(new_n159));
  norp02aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n12x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  xnrc02aa1n02x5               g067(.a(new_n159), .b(new_n162), .out0(\s[14] ));
  nor042aa1n03x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  tech160nm_finand02aa1n03p5x5 g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nano23aa1n06x5               g071(.a(new_n154), .b(new_n160), .c(new_n161), .d(new_n155), .out0(new_n167));
  tech160nm_fiao0012aa1n03p5x5 g072(.a(new_n160), .b(new_n154), .c(new_n161), .o(new_n168));
  aoai13aa1n06x5               g073(.a(new_n166), .b(new_n168), .c(new_n158), .d(new_n167), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n166), .b(new_n168), .c(new_n158), .d(new_n167), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  nor042aa1n03x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  tech160nm_finand02aa1n03p5x5 g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  nona22aa1n02x5               g079(.a(new_n169), .b(new_n174), .c(new_n164), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n174), .o1(new_n176));
  oaoi13aa1n06x5               g081(.a(new_n176), .b(new_n169), .c(\a[15] ), .d(\b[14] ), .o1(new_n177));
  norb02aa1n03x4               g082(.a(new_n175), .b(new_n177), .out0(\s[16] ));
  norp02aa1n02x5               g083(.a(new_n100), .b(new_n97), .o1(new_n179));
  nona23aa1n03x5               g084(.a(new_n146), .b(new_n145), .c(new_n179), .d(new_n129), .out0(new_n180));
  nand23aa1n06x5               g085(.a(new_n180), .b(new_n142), .c(new_n149), .o1(new_n181));
  nano23aa1n09x5               g086(.a(new_n164), .b(new_n172), .c(new_n173), .d(new_n165), .out0(new_n182));
  nand22aa1n03x5               g087(.a(new_n182), .b(new_n167), .o1(new_n183));
  inv020aa1n02x5               g088(.a(new_n183), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n185));
  nand02aa1n03x5               g090(.a(new_n182), .b(new_n168), .o1(new_n186));
  nona22aa1n03x5               g091(.a(new_n186), .b(new_n185), .c(new_n172), .out0(new_n187));
  aoi012aa1d18x5               g092(.a(new_n187), .b(new_n181), .c(new_n184), .o1(new_n188));
  nano23aa1n06x5               g093(.a(new_n97), .b(new_n100), .c(new_n127), .d(new_n98), .out0(new_n189));
  nano32aa1n03x7               g094(.a(new_n183), .b(new_n189), .c(new_n146), .d(new_n145), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n125), .c(new_n131), .d(new_n130), .o1(new_n191));
  nanp02aa1n06x5               g096(.a(new_n188), .b(new_n191), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g098(.a(\a[18] ), .o1(new_n194));
  inv040aa1d30x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  oaoi03aa1n03x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  xroi22aa1d06x4               g103(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nor002aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand22aa1n06x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  aoi013aa1n06x4               g107(.a(new_n201), .b(new_n202), .c(new_n195), .d(new_n196), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n200), .c(new_n188), .d(new_n191), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nand02aa1d28x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  nor002aa1d32x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1d24x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  aoi112aa1n03x5               g116(.a(new_n207), .b(new_n211), .c(new_n204), .d(new_n208), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n211), .b(new_n207), .c(new_n204), .d(new_n208), .o1(new_n213));
  norb02aa1n03x4               g118(.a(new_n213), .b(new_n212), .out0(\s[20] ));
  nano23aa1n09x5               g119(.a(new_n207), .b(new_n209), .c(new_n210), .d(new_n208), .out0(new_n215));
  nand02aa1d04x5               g120(.a(new_n199), .b(new_n215), .o1(new_n216));
  nona23aa1n09x5               g121(.a(new_n210), .b(new_n208), .c(new_n207), .d(new_n209), .out0(new_n217));
  oai012aa1n04x7               g122(.a(new_n210), .b(new_n209), .c(new_n207), .o1(new_n218));
  oai012aa1n12x5               g123(.a(new_n218), .b(new_n217), .c(new_n203), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n216), .c(new_n188), .d(new_n191), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n02x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n03x4               g132(.a(new_n227), .b(new_n226), .out0(\s[22] ));
  inv030aa1d32x5               g133(.a(\a[21] ), .o1(new_n229));
  inv040aa1d32x5               g134(.a(\a[22] ), .o1(new_n230));
  xroi22aa1d06x4               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nand23aa1n06x5               g136(.a(new_n231), .b(new_n199), .c(new_n215), .o1(new_n232));
  nona22aa1n02x4               g137(.a(new_n202), .b(\b[16] ), .c(\a[17] ), .out0(new_n233));
  oaib12aa1n02x5               g138(.a(new_n233), .b(\b[17] ), .c(new_n194), .out0(new_n234));
  inv020aa1n02x5               g139(.a(new_n218), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n231), .b(new_n235), .c(new_n215), .d(new_n234), .o1(new_n236));
  inv000aa1d42x5               g141(.a(\b[21] ), .o1(new_n237));
  oao003aa1n02x5               g142(.a(new_n230), .b(new_n237), .c(new_n223), .carry(new_n238));
  inv020aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n236), .b(new_n239), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n232), .c(new_n188), .d(new_n191), .o1(new_n242));
  xorb03aa1n02x5               g147(.a(new_n242), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  xorc02aa1n12x5               g150(.a(\a[24] ), .b(\b[23] ), .out0(new_n246));
  aoi112aa1n03x5               g151(.a(new_n244), .b(new_n246), .c(new_n242), .d(new_n245), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n245), .o1(new_n248));
  norb02aa1n03x4               g153(.a(new_n248), .b(new_n247), .out0(\s[24] ));
  and002aa1n02x5               g154(.a(new_n246), .b(new_n245), .o(new_n250));
  nanb03aa1n03x5               g155(.a(new_n216), .b(new_n250), .c(new_n231), .out0(new_n251));
  inv020aa1n02x5               g156(.a(new_n250), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  oai022aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(new_n254), .b(new_n253), .o1(new_n255));
  aoai13aa1n12x5               g160(.a(new_n255), .b(new_n252), .c(new_n236), .d(new_n239), .o1(new_n256));
  inv000aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n04x5               g162(.a(new_n257), .b(new_n251), .c(new_n188), .d(new_n191), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  tech160nm_fixorc02aa1n02p5x5 g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n264));
  norb02aa1n02x7               g169(.a(new_n264), .b(new_n263), .out0(\s[26] ));
  oabi12aa1n03x5               g170(.a(new_n187), .b(new_n153), .c(new_n183), .out0(new_n266));
  and002aa1n06x5               g171(.a(new_n262), .b(new_n261), .o(new_n267));
  nano22aa1n03x7               g172(.a(new_n232), .b(new_n250), .c(new_n267), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n266), .c(new_n126), .d(new_n190), .o1(new_n269));
  oai022aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n270));
  aob012aa1n02x5               g175(.a(new_n270), .b(\b[25] ), .c(\a[26] ), .out0(new_n271));
  aobi12aa1n12x5               g176(.a(new_n271), .b(new_n256), .c(new_n267), .out0(new_n272));
  xorc02aa1n12x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  inv040aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  aobi12aa1n02x7               g181(.a(new_n273), .b(new_n272), .c(new_n269), .out0(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n277), .b(new_n276), .c(new_n278), .out0(new_n279));
  inv020aa1n02x5               g184(.a(new_n268), .o1(new_n280));
  aoi012aa1n06x5               g185(.a(new_n280), .b(new_n188), .c(new_n191), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n250), .b(new_n238), .c(new_n219), .d(new_n231), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n267), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n271), .b(new_n283), .c(new_n282), .d(new_n255), .o1(new_n284));
  oaih12aa1n02x5               g189(.a(new_n273), .b(new_n284), .c(new_n281), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n278), .b(new_n285), .c(new_n276), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n286), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n273), .b(new_n278), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n284), .c(new_n281), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n288), .b(new_n272), .c(new_n269), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n117), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n273), .b(new_n291), .c(new_n278), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n297), .b(new_n284), .c(new_n281), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n03p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n297), .b(new_n272), .c(new_n269), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n297), .b(new_n300), .out0(new_n305));
  aobi12aa1n02x7               g210(.a(new_n305), .b(new_n272), .c(new_n269), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  oaih12aa1n02x5               g214(.a(new_n305), .b(new_n284), .c(new_n281), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n119), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n119), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n131), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g221(.a(new_n107), .b(new_n131), .c(new_n108), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g223(.a(new_n105), .b(new_n107), .c(new_n106), .o(new_n319));
  oabi12aa1n02x5               g224(.a(new_n319), .b(new_n121), .c(new_n109), .out0(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  orn002aa1n02x5               g226(.a(\a[7] ), .b(\b[6] ), .o(new_n322));
  nanp02aa1n02x5               g227(.a(new_n320), .b(new_n104), .o1(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n103), .b(new_n323), .c(new_n322), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


