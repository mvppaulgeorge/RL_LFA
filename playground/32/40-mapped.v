// Benchmark "adder" written by ABC on Thu Jul 18 04:43:30 2024

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
    new_n140, new_n141, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n321, new_n324, new_n325, new_n327,
    new_n328, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv000aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n06x5               g006(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(new_n102));
  nor042aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanb02aa1n02x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  oaih22aa1d12x5               g010(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n106));
  inv000aa1d42x5               g011(.a(new_n106), .o1(new_n107));
  aoai13aa1n04x5               g012(.a(new_n107), .b(new_n105), .c(new_n100), .d(new_n102), .o1(new_n108));
  nor022aa1n04x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  norp02aa1n04x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n08x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nona23aa1n08x5               g017(.a(new_n111), .b(new_n112), .c(new_n110), .d(new_n109), .out0(new_n113));
  inv000aa1n02x5               g018(.a(new_n113), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  oai012aa1n02x5               g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n118), .b(new_n116), .c(new_n119), .d(new_n115), .out0(new_n120));
  nand23aa1n03x5               g025(.a(new_n108), .b(new_n114), .c(new_n120), .o1(new_n121));
  norb03aa1n03x4               g026(.a(new_n111), .b(new_n109), .c(new_n110), .out0(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  aoi022aa1n02x5               g028(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n124));
  inv000aa1d42x5               g029(.a(\a[7] ), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(\b[6] ), .b(new_n125), .out0(new_n126));
  oaoi03aa1n02x5               g031(.a(\a[8] ), .b(\b[7] ), .c(new_n126), .o1(new_n127));
  aoi013aa1n06x4               g032(.a(new_n127), .b(new_n122), .c(new_n123), .d(new_n124), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nanb02aa1n06x5               g034(.a(new_n97), .b(new_n129), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n98), .b(new_n130), .c(new_n121), .d(new_n128), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  oai022aa1n02x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  norp02aa1n02x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n135), .b(new_n133), .out0(new_n136));
  aoi112aa1n02x5               g041(.a(new_n136), .b(new_n130), .c(new_n121), .d(new_n128), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n137), .b(new_n133), .c(new_n134), .o1(new_n138));
  inv000aa1d42x5               g043(.a(\b[10] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(\a[11] ), .b(new_n139), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n138), .b(new_n141), .c(new_n140), .out0(\s[11] ));
  oaoi03aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .c(new_n138), .o1(new_n143));
  xorb03aa1n02x5               g048(.a(new_n143), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nano22aa1n02x4               g051(.a(new_n145), .b(new_n141), .c(new_n146), .out0(new_n147));
  nona23aa1n02x4               g052(.a(new_n140), .b(new_n147), .c(new_n136), .d(new_n130), .out0(new_n148));
  nanb03aa1n02x5               g053(.a(new_n145), .b(new_n146), .c(new_n141), .out0(new_n149));
  nano32aa1n02x5               g054(.a(new_n149), .b(new_n134), .c(new_n140), .d(new_n133), .out0(new_n150));
  oaoi03aa1n03x5               g055(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n151));
  norp02aa1n02x5               g056(.a(new_n150), .b(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n148), .c(new_n121), .d(new_n128), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand42aa1n08x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  inv030aa1d32x5               g062(.a(\a[13] ), .o1(new_n158));
  inv030aa1d32x5               g063(.a(\b[12] ), .o1(new_n159));
  oaoi03aa1n02x5               g064(.a(new_n158), .b(new_n159), .c(new_n153), .o1(new_n160));
  xnrc02aa1n02x5               g065(.a(new_n160), .b(new_n157), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  aoai13aa1n12x5               g069(.a(new_n156), .b(new_n155), .c(new_n158), .d(new_n159), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  aoi013aa1n06x4               g071(.a(new_n166), .b(new_n153), .c(new_n164), .d(new_n157), .o1(new_n167));
  nor042aa1n04x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nand42aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n167), .b(new_n170), .c(new_n169), .out0(\s[15] ));
  nanb02aa1n02x5               g076(.a(new_n168), .b(new_n170), .out0(new_n172));
  nor042aa1n06x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  oai112aa1n02x5               g080(.a(new_n175), .b(new_n169), .c(new_n167), .d(new_n172), .o1(new_n176));
  oaoi13aa1n02x5               g081(.a(new_n175), .b(new_n169), .c(new_n167), .d(new_n172), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(\s[16] ));
  nano32aa1n02x4               g083(.a(new_n136), .b(new_n98), .c(new_n140), .d(new_n129), .out0(new_n179));
  norb02aa1n06x4               g084(.a(new_n170), .b(new_n168), .out0(new_n180));
  norb02aa1n06x4               g085(.a(new_n174), .b(new_n173), .out0(new_n181));
  nona23aa1n02x4               g086(.a(new_n156), .b(new_n163), .c(new_n162), .d(new_n155), .out0(new_n182));
  nano22aa1n03x7               g087(.a(new_n182), .b(new_n180), .c(new_n181), .out0(new_n183));
  nand03aa1n02x5               g088(.a(new_n183), .b(new_n179), .c(new_n147), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n173), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n174), .b(new_n170), .o1(new_n186));
  aoai13aa1n09x5               g091(.a(new_n185), .b(new_n186), .c(new_n165), .d(new_n169), .o1(new_n187));
  oaoi13aa1n09x5               g092(.a(new_n187), .b(new_n183), .c(new_n150), .d(new_n151), .o1(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n184), .c(new_n121), .d(new_n128), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  aoi022aa1n02x5               g095(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n191));
  norb02aa1n02x7               g096(.a(new_n104), .b(new_n103), .out0(new_n192));
  oaoi13aa1n02x5               g097(.a(new_n106), .b(new_n192), .c(new_n191), .d(new_n99), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n115), .b(new_n116), .out0(new_n194));
  and002aa1n02x5               g099(.a(\b[5] ), .b(\a[6] ), .o(new_n195));
  nona22aa1n02x4               g100(.a(new_n194), .b(new_n118), .c(new_n195), .out0(new_n196));
  oai013aa1n03x4               g101(.a(new_n128), .b(new_n193), .c(new_n113), .d(new_n196), .o1(new_n197));
  norb02aa1n03x5               g102(.a(new_n183), .b(new_n148), .out0(new_n198));
  oai012aa1n02x5               g103(.a(new_n183), .b(new_n150), .c(new_n151), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n187), .b(new_n199), .out0(new_n200));
  tech160nm_fiaoi012aa1n05x5   g105(.a(new_n200), .b(new_n197), .c(new_n198), .o1(new_n201));
  oaoi03aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(new_n201), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g108(.a(\a[18] ), .o1(new_n204));
  inv000aa1d42x5               g109(.a(\b[17] ), .o1(new_n205));
  oai022aa1n03x5               g110(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  oa0012aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n204), .o(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  tech160nm_fixorc02aa1n02p5x5 g113(.a(\a[17] ), .b(\b[16] ), .out0(new_n209));
  xorc02aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .out0(new_n210));
  nanp03aa1n03x5               g115(.a(new_n189), .b(new_n209), .c(new_n210), .o1(new_n211));
  inv040aa1d32x5               g116(.a(\a[19] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\b[18] ), .o1(new_n213));
  nand02aa1n03x5               g118(.a(new_n213), .b(new_n212), .o1(new_n214));
  nanp02aa1n02x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n214), .b(new_n215), .o1(new_n216));
  xobna2aa1n03x5               g121(.a(new_n216), .b(new_n211), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_fiaoi012aa1n04x5   g123(.a(new_n216), .b(new_n211), .c(new_n208), .o1(new_n219));
  nor002aa1n03x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand42aa1n02x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  nano22aa1n03x7               g128(.a(new_n219), .b(new_n214), .c(new_n223), .out0(new_n224));
  aoi013aa1n02x4               g129(.a(new_n207), .b(new_n189), .c(new_n209), .d(new_n210), .o1(new_n225));
  oaoi13aa1n02x5               g130(.a(new_n223), .b(new_n214), .c(new_n225), .d(new_n216), .o1(new_n226));
  norp02aa1n02x5               g131(.a(new_n226), .b(new_n224), .o1(\s[20] ));
  nanb03aa1n03x5               g132(.a(new_n220), .b(new_n221), .c(new_n215), .out0(new_n228));
  nano32aa1n02x5               g133(.a(new_n228), .b(new_n210), .c(new_n209), .d(new_n214), .out0(new_n229));
  oai112aa1n03x5               g134(.a(new_n206), .b(new_n214), .c(new_n205), .d(new_n204), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n221), .b(new_n220), .c(new_n212), .d(new_n213), .o1(new_n231));
  oai012aa1n06x5               g136(.a(new_n231), .b(new_n230), .c(new_n228), .o1(new_n232));
  tech160nm_fixorc02aa1n03p5x5 g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(new_n189), .d(new_n229), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n233), .b(new_n232), .c(new_n189), .d(new_n229), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(\s[21] ));
  inv000aa1d42x5               g141(.a(\a[21] ), .o1(new_n237));
  nanb02aa1n02x5               g142(.a(\b[20] ), .b(new_n237), .out0(new_n238));
  tech160nm_fixorc02aa1n03p5x5 g143(.a(\a[22] ), .b(\b[21] ), .out0(new_n239));
  xnbna2aa1n03x5               g144(.a(new_n239), .b(new_n234), .c(new_n238), .out0(\s[22] ));
  inv000aa1d42x5               g145(.a(\a[22] ), .o1(new_n241));
  xroi22aa1d04x5               g146(.a(new_n237), .b(\b[20] ), .c(new_n241), .d(\b[21] ), .out0(new_n242));
  and002aa1n02x5               g147(.a(new_n229), .b(new_n242), .o(new_n243));
  oaoi03aa1n12x5               g148(.a(\a[22] ), .b(\b[21] ), .c(new_n238), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aob012aa1n02x5               g150(.a(new_n245), .b(new_n232), .c(new_n242), .out0(new_n246));
  xnrc02aa1n12x5               g151(.a(\b[22] ), .b(\a[23] ), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n189), .d(new_n243), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n246), .c(new_n189), .d(new_n243), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[23] ));
  orn002aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .o(new_n252));
  xnrc02aa1n02x5               g157(.a(\b[23] ), .b(\a[24] ), .out0(new_n253));
  xobna2aa1n03x5               g158(.a(new_n253), .b(new_n249), .c(new_n252), .out0(\s[24] ));
  norp02aa1n02x5               g159(.a(new_n253), .b(new_n247), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n244), .c(new_n232), .d(new_n242), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n252), .carry(new_n257));
  nand42aa1n08x5               g162(.a(new_n256), .b(new_n257), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n229), .o1(new_n260));
  nano23aa1n03x7               g165(.a(new_n253), .b(new_n247), .c(new_n239), .d(new_n233), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nona22aa1n03x5               g167(.a(new_n189), .b(new_n260), .c(new_n262), .out0(new_n263));
  xnrc02aa1n12x5               g168(.a(\b[24] ), .b(\a[25] ), .out0(new_n264));
  xobna2aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n259), .out0(\s[25] ));
  orn002aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .o(new_n266));
  aoi012aa1n02x5               g171(.a(new_n264), .b(new_n263), .c(new_n259), .o1(new_n267));
  xnrc02aa1n06x5               g172(.a(\b[25] ), .b(\a[26] ), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n267), .b(new_n266), .c(new_n268), .out0(new_n269));
  aoi013aa1n02x4               g174(.a(new_n258), .b(new_n189), .c(new_n229), .d(new_n261), .o1(new_n270));
  oaoi13aa1n02x5               g175(.a(new_n268), .b(new_n266), .c(new_n270), .d(new_n264), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n269), .o1(\s[26] ));
  nanp02aa1n02x5               g177(.a(\b[25] ), .b(\a[26] ), .o1(new_n273));
  nor042aa1n03x5               g178(.a(new_n268), .b(new_n264), .o1(new_n274));
  oai022aa1n02x5               g179(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n275));
  aoi022aa1d18x5               g180(.a(new_n258), .b(new_n274), .c(new_n273), .d(new_n275), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n274), .o1(new_n277));
  nona32aa1n09x5               g182(.a(new_n189), .b(new_n277), .c(new_n262), .d(new_n260), .out0(new_n278));
  norp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  norb02aa1n02x5               g185(.a(new_n280), .b(new_n279), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  inv000aa1n06x5               g187(.a(new_n279), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(new_n275), .b(new_n273), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n277), .c(new_n256), .d(new_n257), .o1(new_n285));
  and003aa1n03x7               g190(.a(new_n242), .b(new_n274), .c(new_n255), .o(new_n286));
  nano22aa1n03x7               g191(.a(new_n201), .b(new_n229), .c(new_n286), .out0(new_n287));
  tech160nm_fioai012aa1n02p5x5 g192(.a(new_n280), .b(new_n287), .c(new_n285), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[27] ), .b(\a[28] ), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n288), .c(new_n283), .o1(new_n290));
  aoi022aa1n03x5               g195(.a(new_n276), .b(new_n278), .c(\b[26] ), .d(\a[27] ), .o1(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n283), .c(new_n289), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[28] ));
  nano22aa1n02x4               g198(.a(new_n289), .b(new_n283), .c(new_n280), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n287), .c(new_n285), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n06x5               g203(.a(new_n294), .b(new_n276), .c(new_n278), .out0(new_n299));
  nano22aa1n03x7               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n281), .b(new_n297), .c(new_n289), .out0(new_n303));
  oai012aa1n02x5               g208(.a(new_n303), .b(new_n287), .c(new_n285), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n06x5               g212(.a(new_n303), .b(new_n276), .c(new_n278), .out0(new_n308));
  nano22aa1n03x7               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  norb03aa1n02x5               g216(.a(new_n294), .b(new_n306), .c(new_n297), .out0(new_n312));
  oai012aa1n02x5               g217(.a(new_n312), .b(new_n287), .c(new_n285), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n314));
  aoi012aa1n02x5               g219(.a(new_n311), .b(new_n313), .c(new_n314), .o1(new_n315));
  aobi12aa1n06x5               g220(.a(new_n312), .b(new_n276), .c(new_n278), .out0(new_n316));
  nano22aa1n03x7               g221(.a(new_n316), .b(new_n311), .c(new_n314), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n315), .b(new_n317), .o1(\s[31] ));
  xnbna2aa1n03x5               g223(.a(new_n192), .b(new_n102), .c(new_n100), .out0(\s[3] ));
  xorc02aa1n02x5               g224(.a(\a[4] ), .b(\b[3] ), .out0(new_n320));
  oaoi13aa1n02x5               g225(.a(new_n103), .b(new_n104), .c(new_n191), .d(new_n99), .o1(new_n321));
  mtn022aa1n02x5               g226(.a(new_n108), .b(new_n321), .sa(new_n320), .o1(\s[4] ));
  xobna2aa1n03x5               g227(.a(new_n194), .b(new_n108), .c(new_n112), .out0(\s[5] ));
  aoai13aa1n02x5               g228(.a(new_n194), .b(new_n193), .c(\a[4] ), .d(\b[3] ), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[6] ), .b(\b[5] ), .out0(new_n325));
  xobna2aa1n03x5               g230(.a(new_n325), .b(new_n324), .c(new_n115), .out0(\s[6] ));
  nanp02aa1n02x5               g231(.a(new_n126), .b(new_n111), .o1(new_n327));
  aob012aa1n02x5               g232(.a(new_n325), .b(new_n324), .c(new_n115), .out0(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n327), .b(new_n328), .c(new_n119), .out0(\s[7] ));
  aoi013aa1n02x4               g234(.a(new_n110), .b(new_n328), .c(new_n119), .d(new_n111), .o1(new_n330));
  xnrb03aa1n02x5               g235(.a(new_n330), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g236(.a(new_n130), .b(new_n121), .c(new_n128), .out0(\s[9] ));
endmodule


