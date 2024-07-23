// Benchmark "adder" written by ABC on Wed Jul 17 20:30:57 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n324, new_n326, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv040aa1n08x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  inv040aa1d32x5               g004(.a(\a[3] ), .o1(new_n100));
  inv040aa1n16x5               g005(.a(\b[2] ), .o1(new_n101));
  nanp02aa1n06x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nand02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand02aa1n03x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  nor042aa1n03x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand42aa1n06x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand42aa1n16x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  aoi012aa1n12x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[4] ), .o1(new_n109));
  aboi22aa1n06x5               g014(.a(\b[3] ), .b(new_n109), .c(new_n100), .d(new_n101), .out0(new_n110));
  oaoi13aa1n12x5               g015(.a(new_n99), .b(new_n110), .c(new_n108), .d(new_n104), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor022aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  nor043aa1n03x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n113), .b(new_n115), .c(new_n112), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[6] ), .o1(new_n121));
  oai022aa1n02x7               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  oaib12aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(\b[5] ), .out0(new_n123));
  oai012aa1n06x5               g028(.a(new_n120), .b(new_n116), .c(new_n123), .o1(new_n124));
  nand42aa1n20x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n03x5               g031(.a(new_n126), .b(new_n124), .c(new_n111), .d(new_n119), .o1(new_n127));
  nor002aa1n16x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  aobi12aa1n06x5               g036(.a(new_n130), .b(new_n127), .c(new_n98), .out0(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand02aa1n16x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  oaoi03aa1n02x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n136));
  tech160nm_fioai012aa1n05x5   g041(.a(new_n135), .b(new_n132), .c(new_n136), .o1(new_n137));
  nanb02aa1d24x5               g042(.a(new_n133), .b(new_n134), .out0(new_n138));
  tech160nm_fioai012aa1n05x5   g043(.a(new_n129), .b(new_n128), .c(new_n97), .o1(new_n139));
  nano22aa1n02x4               g044(.a(new_n132), .b(new_n138), .c(new_n139), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n137), .b(new_n140), .out0(\s[11] ));
  nor002aa1d24x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand22aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  nona22aa1n02x4               g049(.a(new_n137), .b(new_n144), .c(new_n133), .out0(new_n145));
  nanb02aa1d24x5               g050(.a(new_n142), .b(new_n143), .out0(new_n146));
  oaoi13aa1n02x5               g051(.a(new_n146), .b(new_n137), .c(\a[11] ), .d(\b[10] ), .o1(new_n147));
  norb02aa1n03x4               g052(.a(new_n145), .b(new_n147), .out0(\s[12] ));
  nano23aa1d15x5               g053(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n149));
  nona22aa1d36x5               g054(.a(new_n149), .b(new_n146), .c(new_n138), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n124), .c(new_n111), .d(new_n119), .o1(new_n152));
  tech160nm_fiaoi012aa1n05x5   g057(.a(new_n142), .b(new_n133), .c(new_n143), .o1(new_n153));
  oai013aa1n09x5               g058(.a(new_n153), .b(new_n139), .c(new_n138), .d(new_n146), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nor042aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n03x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n06x4               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n152), .c(new_n155), .out0(\s[13] ));
  inv000aa1d42x5               g064(.a(\a[13] ), .o1(new_n160));
  inv000aa1d42x5               g065(.a(\b[12] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(new_n152), .b(new_n155), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(new_n160), .b(new_n161), .c(new_n162), .o1(new_n163));
  nor022aa1n16x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  norb02aa1n12x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  xnrc02aa1n02x5               g071(.a(new_n163), .b(new_n166), .out0(\s[14] ));
  nona23aa1n02x4               g072(.a(new_n165), .b(new_n157), .c(new_n156), .d(new_n164), .out0(new_n168));
  aoai13aa1n04x5               g073(.a(new_n165), .b(new_n164), .c(new_n160), .d(new_n161), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n168), .c(new_n152), .d(new_n155), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n20x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n06x4               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  nor002aa1d24x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n06x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n06x4               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoi112aa1n03x5               g082(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n174), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nano23aa1n06x5               g085(.a(new_n172), .b(new_n175), .c(new_n176), .d(new_n173), .out0(new_n181));
  nand23aa1d12x5               g086(.a(new_n181), .b(new_n158), .c(new_n166), .o1(new_n182));
  nor042aa1d18x5               g087(.a(new_n182), .b(new_n150), .o1(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n124), .c(new_n111), .d(new_n119), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n175), .o1(new_n185));
  nanb03aa1n06x5               g090(.a(new_n169), .b(new_n177), .c(new_n174), .out0(new_n186));
  aoi112aa1n03x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  inv000aa1n02x5               g092(.a(new_n187), .o1(new_n188));
  nand03aa1n02x5               g093(.a(new_n136), .b(new_n135), .c(new_n144), .o1(new_n189));
  tech160nm_fiaoi012aa1n05x5   g094(.a(new_n182), .b(new_n189), .c(new_n153), .o1(new_n190));
  nano32aa1d12x5               g095(.a(new_n190), .b(new_n188), .c(new_n186), .d(new_n185), .out0(new_n191));
  nanp02aa1n12x5               g096(.a(new_n191), .b(new_n184), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g098(.a(\a[18] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  oaoi03aa1n03x5               g101(.a(new_n195), .b(new_n196), .c(new_n192), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[17] ), .c(new_n194), .out0(\s[18] ));
  nor002aa1n20x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand02aa1d06x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  xroi22aa1d06x4               g106(.a(new_n195), .b(\b[16] ), .c(new_n194), .d(\b[17] ), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n196), .b(new_n195), .o1(new_n203));
  oaoi03aa1n12x5               g108(.a(\a[18] ), .b(\b[17] ), .c(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n201), .b(new_n204), .c(new_n192), .d(new_n202), .o1(new_n205));
  aoi112aa1n02x5               g110(.a(new_n201), .b(new_n204), .c(new_n192), .d(new_n202), .o1(new_n206));
  norb02aa1n02x7               g111(.a(new_n205), .b(new_n206), .out0(\s[19] ));
  xnrc02aa1n02x5               g112(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n12x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand22aa1n09x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n210), .b(new_n209), .out0(new_n211));
  nona22aa1n02x5               g116(.a(new_n205), .b(new_n211), .c(new_n199), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n199), .o1(new_n213));
  aobi12aa1n06x5               g118(.a(new_n211), .b(new_n205), .c(new_n213), .out0(new_n214));
  norb02aa1n03x4               g119(.a(new_n212), .b(new_n214), .out0(\s[20] ));
  nano23aa1n06x5               g120(.a(new_n199), .b(new_n209), .c(new_n210), .d(new_n200), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n202), .b(new_n216), .o1(new_n217));
  oaih22aa1n04x5               g122(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n218));
  oaib12aa1n06x5               g123(.a(new_n218), .b(new_n194), .c(\b[17] ), .out0(new_n219));
  nona23aa1d18x5               g124(.a(new_n210), .b(new_n200), .c(new_n199), .d(new_n209), .out0(new_n220));
  aoi012aa1d18x5               g125(.a(new_n209), .b(new_n199), .c(new_n210), .o1(new_n221));
  oai012aa1d24x5               g126(.a(new_n221), .b(new_n220), .c(new_n219), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n217), .c(new_n191), .d(new_n184), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoi112aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x7               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\a[22] ), .o1(new_n233));
  xroi22aa1d06x4               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n234), .b(new_n202), .c(new_n216), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\b[21] ), .o1(new_n236));
  oaoi03aa1n12x5               g141(.a(new_n233), .b(new_n236), .c(new_n226), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n222), .c(new_n234), .o1(new_n239));
  aoai13aa1n04x5               g144(.a(new_n239), .b(new_n235), .c(new_n191), .d(new_n184), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  xorc02aa1n03x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n243), .o1(new_n246));
  norb02aa1n02x7               g151(.a(new_n246), .b(new_n245), .out0(\s[24] ));
  and002aa1n02x5               g152(.a(new_n244), .b(new_n243), .o(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n234), .c(new_n202), .d(new_n216), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n221), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n234), .b(new_n251), .c(new_n216), .d(new_n204), .o1(new_n252));
  norp02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  aoi012aa1n02x5               g159(.a(new_n253), .b(new_n242), .c(new_n254), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n249), .c(new_n252), .d(new_n237), .o1(new_n256));
  tech160nm_fixorc02aa1n05x5   g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n256), .c(new_n192), .d(new_n250), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(new_n257), .b(new_n256), .c(new_n192), .d(new_n250), .o1(new_n259));
  norb02aa1n02x7               g164(.a(new_n258), .b(new_n259), .out0(\s[25] ));
  nor042aa1n03x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  tech160nm_fixorc02aa1n04x5   g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  nona22aa1n02x5               g167(.a(new_n258), .b(new_n262), .c(new_n261), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n261), .o1(new_n264));
  aobi12aa1n06x5               g169(.a(new_n262), .b(new_n258), .c(new_n264), .out0(new_n265));
  norb02aa1n03x4               g170(.a(new_n263), .b(new_n265), .out0(\s[26] ));
  oai012aa1n02x5               g171(.a(new_n110), .b(new_n108), .c(new_n104), .o1(new_n267));
  oaib12aa1n06x5               g172(.a(new_n267), .b(new_n109), .c(\b[3] ), .out0(new_n268));
  orn003aa1n02x5               g173(.a(new_n116), .b(new_n118), .c(new_n117), .o(new_n269));
  oabi12aa1n06x5               g174(.a(new_n124), .b(new_n269), .c(new_n268), .out0(new_n270));
  nano22aa1n03x5               g175(.a(new_n169), .b(new_n174), .c(new_n177), .out0(new_n271));
  nanb02aa1n06x5               g176(.a(new_n182), .b(new_n154), .out0(new_n272));
  nona32aa1n03x5               g177(.a(new_n272), .b(new_n187), .c(new_n271), .d(new_n175), .out0(new_n273));
  and002aa1n09x5               g178(.a(new_n262), .b(new_n257), .o(new_n274));
  nano22aa1n03x7               g179(.a(new_n235), .b(new_n248), .c(new_n274), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n273), .c(new_n270), .d(new_n183), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n264), .carry(new_n277));
  aobi12aa1n06x5               g182(.a(new_n277), .b(new_n256), .c(new_n274), .out0(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n278), .c(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  inv040aa1n03x5               g186(.a(new_n281), .o1(new_n282));
  aobi12aa1n02x7               g187(.a(new_n279), .b(new_n278), .c(new_n276), .out0(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n283), .b(new_n282), .c(new_n284), .out0(new_n285));
  aobi12aa1n06x5               g190(.a(new_n275), .b(new_n191), .c(new_n184), .out0(new_n286));
  aoai13aa1n04x5               g191(.a(new_n248), .b(new_n238), .c(new_n222), .d(new_n234), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n274), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n277), .b(new_n288), .c(new_n287), .d(new_n255), .o1(new_n289));
  oaih12aa1n02x5               g194(.a(new_n279), .b(new_n289), .c(new_n286), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n284), .b(new_n290), .c(new_n282), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n285), .o1(\s[28] ));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  norb02aa1n02x5               g198(.a(new_n279), .b(new_n284), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n289), .c(new_n286), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n293), .b(new_n295), .c(new_n296), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n294), .b(new_n278), .c(new_n276), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g206(.a(new_n279), .b(new_n293), .c(new_n284), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n289), .c(new_n286), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x7               g211(.a(new_n302), .b(new_n278), .c(new_n276), .out0(new_n307));
  nano22aa1n02x4               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  norb02aa1n02x5               g215(.a(new_n302), .b(new_n305), .out0(new_n311));
  aobi12aa1n06x5               g216(.a(new_n311), .b(new_n278), .c(new_n276), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n313));
  nano22aa1n03x5               g218(.a(new_n312), .b(new_n310), .c(new_n313), .out0(new_n314));
  oaih12aa1n02x5               g219(.a(new_n311), .b(new_n289), .c(new_n286), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n310), .b(new_n315), .c(new_n313), .o1(new_n316));
  norp02aa1n03x5               g221(.a(new_n316), .b(new_n314), .o1(\s[31] ));
  xnbna2aa1n03x5               g222(.a(new_n108), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n03x5               g226(.a(\a[5] ), .b(\b[4] ), .c(new_n268), .carry(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(new_n121), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g228(.a(\a[6] ), .b(\b[5] ), .c(new_n322), .o1(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  orn002aa1n02x5               g230(.a(\a[8] ), .b(\b[7] ), .o(new_n326));
  tech160nm_fiaoi012aa1n05x5   g231(.a(new_n115), .b(new_n324), .c(new_n114), .o1(new_n327));
  xnbna2aa1n03x5               g232(.a(new_n327), .b(new_n326), .c(new_n113), .out0(\s[8] ));
  xorb03aa1n02x5               g233(.a(new_n270), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


