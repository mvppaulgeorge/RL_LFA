// Benchmark "adder" written by ABC on Thu Jul 18 10:47:22 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n357, new_n358, new_n360,
    new_n362, new_n363, new_n364, new_n365, new_n366, new_n367, new_n369,
    new_n370, new_n371, new_n374, new_n375;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  orn002aa1n24x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  tech160nm_finand02aa1n05x5   g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aob012aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nor022aa1n08x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norb02aa1n03x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  nor002aa1d24x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  norb02aa1n03x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nanp03aa1n06x5               g012(.a(new_n101), .b(new_n104), .c(new_n107), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n103), .b(new_n105), .c(new_n102), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n08x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  norb02aa1n02x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nand42aa1n08x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  aob012aa1n02x5               g018(.a(new_n113), .b(\b[4] ), .c(\a[5] ), .out0(new_n114));
  oaih22aa1n06x5               g019(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n115));
  xorc02aa1n12x5               g020(.a(\a[8] ), .b(\b[7] ), .out0(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n112), .c(new_n114), .d(new_n115), .out0(new_n117));
  nano22aa1n02x5               g022(.a(new_n110), .b(new_n111), .c(new_n113), .out0(new_n118));
  inv000aa1d42x5               g023(.a(new_n110), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  aoi013aa1n06x4               g025(.a(new_n120), .b(new_n118), .c(new_n116), .d(new_n115), .o1(new_n121));
  aoai13aa1n12x5               g026(.a(new_n121), .b(new_n117), .c(new_n108), .d(new_n109), .o1(new_n122));
  nand42aa1n06x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n97), .b(new_n122), .c(new_n123), .o1(new_n124));
  nor042aa1n09x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n10x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  norb02aa1n02x5               g032(.a(new_n123), .b(new_n97), .out0(new_n128));
  norb03aa1n02x5               g033(.a(new_n126), .b(new_n97), .c(new_n125), .out0(new_n129));
  aob012aa1n02x5               g034(.a(new_n129), .b(new_n122), .c(new_n128), .out0(new_n130));
  oai012aa1n02x5               g035(.a(new_n130), .b(new_n124), .c(new_n127), .o1(\s[10] ));
  nanp03aa1n02x5               g036(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n132));
  nona23aa1n02x4               g037(.a(new_n106), .b(new_n103), .c(new_n102), .d(new_n105), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n109), .b(new_n133), .c(new_n132), .d(new_n98), .o1(new_n134));
  aoi022aa1n02x5               g039(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n135));
  inv000aa1n02x5               g040(.a(new_n115), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n116), .o1(new_n137));
  nano32aa1n03x7               g042(.a(new_n137), .b(new_n112), .c(new_n136), .d(new_n135), .out0(new_n138));
  nand43aa1n02x5               g043(.a(new_n118), .b(new_n116), .c(new_n115), .o1(new_n139));
  nanb02aa1n06x5               g044(.a(new_n120), .b(new_n139), .out0(new_n140));
  nano23aa1n09x5               g045(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n123), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n140), .c(new_n134), .d(new_n138), .o1(new_n142));
  oai012aa1n02x5               g047(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n143));
  nor042aa1n06x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nand02aa1n08x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  norb02aa1n06x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n142), .c(new_n143), .out0(\s[11] ));
  inv040aa1n03x5               g052(.a(new_n144), .o1(new_n148));
  aob012aa1n02x5               g053(.a(new_n146), .b(new_n142), .c(new_n143), .out0(new_n149));
  nor022aa1n08x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  nand42aa1n10x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  norb02aa1n12x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  nona23aa1n02x4               g057(.a(new_n149), .b(new_n151), .c(new_n150), .d(new_n144), .out0(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n152), .c(new_n148), .d(new_n149), .o1(\s[12] ));
  nand23aa1d12x5               g059(.a(new_n141), .b(new_n146), .c(new_n152), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nanb03aa1n02x5               g061(.a(new_n150), .b(new_n151), .c(new_n145), .out0(new_n157));
  oai112aa1n03x5               g062(.a(new_n148), .b(new_n126), .c(new_n125), .d(new_n97), .o1(new_n158));
  oaoi03aa1n09x5               g063(.a(\a[12] ), .b(\b[11] ), .c(new_n148), .o1(new_n159));
  oabi12aa1n06x5               g064(.a(new_n159), .b(new_n157), .c(new_n158), .out0(new_n160));
  nor042aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n16x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n160), .c(new_n122), .d(new_n156), .o1(new_n164));
  nano22aa1n02x4               g069(.a(new_n150), .b(new_n145), .c(new_n151), .out0(new_n165));
  oai012aa1n02x5               g070(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .o1(new_n166));
  oab012aa1n03x5               g071(.a(new_n166), .b(new_n97), .c(new_n125), .out0(new_n167));
  aoi112aa1n02x5               g072(.a(new_n159), .b(new_n163), .c(new_n167), .d(new_n165), .o1(new_n168));
  aobi12aa1n02x5               g073(.a(new_n168), .b(new_n122), .c(new_n156), .out0(new_n169));
  norb02aa1n02x5               g074(.a(new_n164), .b(new_n169), .out0(\s[13] ));
  inv040aa1n03x5               g075(.a(new_n161), .o1(new_n171));
  nor042aa1n04x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n08x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  nona23aa1n03x5               g079(.a(new_n164), .b(new_n173), .c(new_n172), .d(new_n161), .out0(new_n175));
  aoai13aa1n03x5               g080(.a(new_n175), .b(new_n174), .c(new_n171), .d(new_n164), .o1(\s[14] ));
  nano23aa1d15x5               g081(.a(new_n161), .b(new_n172), .c(new_n173), .d(new_n162), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  nona22aa1n06x5               g083(.a(new_n122), .b(new_n155), .c(new_n178), .out0(new_n179));
  tech160nm_fioaoi03aa1n03p5x5 g084(.a(\a[14] ), .b(\b[13] ), .c(new_n171), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n180), .b(new_n160), .c(new_n177), .o1(new_n181));
  nor002aa1n16x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand42aa1d28x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aob012aa1n03x5               g089(.a(new_n184), .b(new_n179), .c(new_n181), .out0(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n180), .c(new_n160), .d(new_n177), .o1(new_n186));
  aobi12aa1n02x7               g091(.a(new_n185), .b(new_n186), .c(new_n179), .out0(\s[15] ));
  inv000aa1d42x5               g092(.a(new_n182), .o1(new_n188));
  nor042aa1n04x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nand42aa1d28x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  nona23aa1n03x5               g096(.a(new_n185), .b(new_n190), .c(new_n189), .d(new_n182), .out0(new_n192));
  aoai13aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n188), .d(new_n185), .o1(\s[16] ));
  nano23aa1d12x5               g098(.a(new_n182), .b(new_n189), .c(new_n190), .d(new_n183), .out0(new_n194));
  nano22aa1d15x5               g099(.a(new_n155), .b(new_n177), .c(new_n194), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n140), .c(new_n134), .d(new_n138), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n194), .b(new_n180), .c(new_n160), .d(new_n177), .o1(new_n197));
  oai012aa1n02x5               g102(.a(new_n190), .b(new_n189), .c(new_n182), .o1(new_n198));
  nand23aa1n06x5               g103(.a(new_n196), .b(new_n197), .c(new_n198), .o1(new_n199));
  xorc02aa1n12x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  nano22aa1n02x4               g105(.a(new_n200), .b(new_n197), .c(new_n198), .out0(new_n201));
  aoi022aa1n02x5               g106(.a(new_n201), .b(new_n196), .c(new_n199), .d(new_n200), .o1(\s[17] ));
  nor002aa1d32x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n04x5               g109(.a(new_n177), .b(new_n159), .c(new_n167), .d(new_n165), .o1(new_n205));
  inv000aa1n02x5               g110(.a(new_n180), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n194), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n198), .b(new_n207), .c(new_n205), .d(new_n206), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n200), .b(new_n208), .c(new_n122), .d(new_n195), .o1(new_n209));
  nor002aa1n03x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nand42aa1n16x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  norb02aa1n03x4               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  oai022aa1d24x5               g117(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n213));
  nanb03aa1n03x5               g118(.a(new_n213), .b(new_n209), .c(new_n211), .out0(new_n214));
  aoai13aa1n03x5               g119(.a(new_n214), .b(new_n212), .c(new_n204), .d(new_n209), .o1(\s[18] ));
  and002aa1n02x5               g120(.a(new_n200), .b(new_n212), .o(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n208), .c(new_n122), .d(new_n195), .o1(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  nor002aa1d32x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nand42aa1n10x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  norb02aa1n12x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g128(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g129(.a(new_n220), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n222), .b(new_n218), .c(new_n199), .d(new_n216), .o1(new_n226));
  nor042aa1n09x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand42aa1d28x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n222), .o1(new_n230));
  norb03aa1n02x5               g135(.a(new_n228), .b(new_n220), .c(new_n227), .out0(new_n231));
  aoai13aa1n02x7               g136(.a(new_n231), .b(new_n230), .c(new_n217), .d(new_n219), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n229), .c(new_n226), .d(new_n225), .o1(\s[20] ));
  nano32aa1n03x7               g138(.a(new_n230), .b(new_n200), .c(new_n229), .d(new_n212), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n208), .c(new_n122), .d(new_n195), .o1(new_n235));
  nanb03aa1n12x5               g140(.a(new_n227), .b(new_n228), .c(new_n221), .out0(new_n236));
  oai112aa1n06x5               g141(.a(new_n213), .b(new_n211), .c(\b[18] ), .d(\a[19] ), .o1(new_n237));
  oai012aa1n12x5               g142(.a(new_n228), .b(new_n227), .c(new_n220), .o1(new_n238));
  oai012aa1n18x5               g143(.a(new_n238), .b(new_n237), .c(new_n236), .o1(new_n239));
  nor002aa1d32x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  nanp02aa1n06x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  norb02aa1n09x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n239), .c(new_n199), .d(new_n234), .o1(new_n243));
  nano22aa1n02x5               g148(.a(new_n227), .b(new_n221), .c(new_n228), .out0(new_n244));
  tech160nm_fioai012aa1n03p5x5 g149(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .o1(new_n245));
  oab012aa1n06x5               g150(.a(new_n245), .b(new_n203), .c(new_n210), .out0(new_n246));
  inv020aa1n02x5               g151(.a(new_n238), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n247), .b(new_n242), .c(new_n246), .d(new_n244), .o1(new_n248));
  aobi12aa1n02x7               g153(.a(new_n243), .b(new_n248), .c(new_n235), .out0(\s[21] ));
  inv040aa1n08x5               g154(.a(new_n240), .o1(new_n250));
  nor042aa1n02x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  nand02aa1n06x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n239), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n242), .o1(new_n255));
  norb03aa1n02x5               g160(.a(new_n252), .b(new_n240), .c(new_n251), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n255), .c(new_n235), .d(new_n254), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n253), .c(new_n243), .d(new_n250), .o1(\s[22] ));
  inv000aa1n02x5               g163(.a(new_n234), .o1(new_n259));
  nano22aa1n06x5               g164(.a(new_n259), .b(new_n242), .c(new_n253), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n208), .c(new_n122), .d(new_n195), .o1(new_n261));
  nano23aa1n09x5               g166(.a(new_n240), .b(new_n251), .c(new_n252), .d(new_n241), .out0(new_n262));
  oaoi03aa1n09x5               g167(.a(\a[22] ), .b(\b[21] ), .c(new_n250), .o1(new_n263));
  aoi012aa1n02x5               g168(.a(new_n263), .b(new_n239), .c(new_n262), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[23] ), .b(\b[22] ), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n265), .c(new_n199), .d(new_n260), .o1(new_n267));
  aoi112aa1n02x5               g172(.a(new_n266), .b(new_n263), .c(new_n239), .d(new_n262), .o1(new_n268));
  aobi12aa1n02x7               g173(.a(new_n267), .b(new_n268), .c(new_n261), .out0(\s[23] ));
  norp02aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n266), .o1(new_n273));
  oai022aa1n02x5               g178(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n274));
  aoi012aa1n02x5               g179(.a(new_n274), .b(\a[24] ), .c(\b[23] ), .o1(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n273), .c(new_n261), .d(new_n264), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n272), .c(new_n267), .d(new_n271), .o1(\s[24] ));
  and002aa1n12x5               g182(.a(new_n272), .b(new_n266), .o(new_n278));
  nano22aa1n02x5               g183(.a(new_n259), .b(new_n278), .c(new_n262), .out0(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n208), .c(new_n122), .d(new_n195), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n262), .b(new_n247), .c(new_n246), .d(new_n244), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n263), .o1(new_n282));
  inv000aa1n09x5               g187(.a(new_n278), .o1(new_n283));
  aob012aa1n02x5               g188(.a(new_n274), .b(\b[23] ), .c(\a[24] ), .out0(new_n284));
  aoai13aa1n12x5               g189(.a(new_n284), .b(new_n283), .c(new_n281), .d(new_n282), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[25] ), .b(\b[24] ), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n199), .d(new_n279), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n278), .b(new_n263), .c(new_n239), .d(new_n262), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n286), .o1(new_n289));
  and003aa1n02x5               g194(.a(new_n288), .b(new_n289), .c(new_n284), .o(new_n290));
  aobi12aa1n02x7               g195(.a(new_n287), .b(new_n290), .c(new_n280), .out0(\s[25] ));
  norp02aa1n02x5               g196(.a(\b[24] ), .b(\a[25] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  tech160nm_fixorc02aa1n02p5x5 g198(.a(\a[26] ), .b(\b[25] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n285), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(\b[25] ), .b(\a[26] ), .o1(new_n296));
  oai022aa1n02x5               g201(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n297));
  norb02aa1n02x5               g202(.a(new_n296), .b(new_n297), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n289), .c(new_n280), .d(new_n295), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n294), .c(new_n287), .d(new_n293), .o1(\s[26] ));
  and002aa1n18x5               g205(.a(new_n294), .b(new_n286), .o(new_n301));
  nano32aa1n06x5               g206(.a(new_n259), .b(new_n301), .c(new_n262), .d(new_n278), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n208), .c(new_n122), .d(new_n195), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n301), .o1(new_n304));
  nanp02aa1n02x5               g209(.a(new_n297), .b(new_n296), .o1(new_n305));
  aoai13aa1n04x5               g210(.a(new_n305), .b(new_n304), .c(new_n288), .d(new_n284), .o1(new_n306));
  xorc02aa1n12x5               g211(.a(\a[27] ), .b(\b[26] ), .out0(new_n307));
  aoai13aa1n06x5               g212(.a(new_n307), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n308));
  aoi122aa1n02x5               g213(.a(new_n307), .b(new_n296), .c(new_n297), .d(new_n285), .e(new_n301), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n308), .b(new_n309), .c(new_n303), .out0(\s[27] ));
  inv000aa1d42x5               g215(.a(\a[27] ), .o1(new_n311));
  inv000aa1d42x5               g216(.a(\b[26] ), .o1(new_n312));
  nanp02aa1n02x5               g217(.a(new_n312), .b(new_n311), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[28] ), .b(\b[27] ), .out0(new_n314));
  aoi022aa1n09x5               g219(.a(new_n285), .b(new_n301), .c(new_n296), .d(new_n297), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n307), .o1(new_n316));
  inv000aa1d42x5               g221(.a(\a[28] ), .o1(new_n317));
  inv000aa1d42x5               g222(.a(\b[27] ), .o1(new_n318));
  aboi22aa1d24x5               g223(.a(\b[27] ), .b(new_n317), .c(new_n311), .d(new_n312), .out0(new_n319));
  oa0012aa1n02x5               g224(.a(new_n319), .b(new_n318), .c(new_n317), .o(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n303), .d(new_n315), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n314), .c(new_n308), .d(new_n313), .o1(\s[28] ));
  xroi22aa1d04x5               g227(.a(new_n311), .b(\b[26] ), .c(new_n317), .d(\b[27] ), .out0(new_n323));
  aoai13aa1n02x5               g228(.a(new_n323), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n323), .o1(new_n325));
  oao003aa1n02x5               g230(.a(\a[28] ), .b(\b[27] ), .c(new_n313), .carry(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n303), .d(new_n315), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[29] ), .b(\b[28] ), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n319), .o1(new_n329));
  oaoi13aa1n02x5               g234(.a(new_n328), .b(new_n329), .c(new_n317), .d(new_n318), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n327), .b(new_n328), .c(new_n324), .d(new_n330), .o1(\s[29] ));
  xorb03aa1n02x5               g236(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g237(.a(new_n316), .b(new_n314), .c(new_n328), .out0(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n333), .o1(new_n335));
  aoi022aa1n02x5               g240(.a(\b[28] ), .b(\a[29] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n336));
  obai22aa1n09x5               g241(.a(new_n336), .b(new_n319), .c(\a[29] ), .d(\b[28] ), .out0(new_n337));
  inv000aa1d42x5               g242(.a(new_n337), .o1(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n335), .c(new_n303), .d(new_n315), .o1(new_n339));
  xorc02aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .out0(new_n340));
  norp02aa1n02x5               g245(.a(\b[28] ), .b(\a[29] ), .o1(new_n341));
  aoi112aa1n02x5               g246(.a(new_n341), .b(new_n340), .c(new_n329), .d(new_n336), .o1(new_n342));
  aoi022aa1n03x5               g247(.a(new_n339), .b(new_n340), .c(new_n334), .d(new_n342), .o1(\s[30] ));
  nano32aa1n06x5               g248(.a(new_n316), .b(new_n340), .c(new_n314), .d(new_n328), .out0(new_n344));
  aoai13aa1n02x5               g249(.a(new_n344), .b(new_n306), .c(new_n199), .d(new_n302), .o1(new_n345));
  inv000aa1d42x5               g250(.a(new_n344), .o1(new_n346));
  inv000aa1d42x5               g251(.a(\a[30] ), .o1(new_n347));
  inv000aa1d42x5               g252(.a(\b[29] ), .o1(new_n348));
  tech160nm_fioaoi03aa1n03p5x5 g253(.a(new_n347), .b(new_n348), .c(new_n337), .o1(new_n349));
  aoai13aa1n03x5               g254(.a(new_n349), .b(new_n346), .c(new_n303), .d(new_n315), .o1(new_n350));
  xorc02aa1n02x5               g255(.a(\a[31] ), .b(\b[30] ), .out0(new_n351));
  aboi22aa1n03x5               g256(.a(\b[30] ), .b(\a[31] ), .c(new_n347), .d(new_n348), .out0(new_n352));
  oaib12aa1n02x5               g257(.a(new_n352), .b(\a[31] ), .c(\b[30] ), .out0(new_n353));
  oaoi13aa1n02x5               g258(.a(new_n353), .b(new_n337), .c(new_n347), .d(new_n348), .o1(new_n354));
  aoi022aa1n03x5               g259(.a(new_n350), .b(new_n351), .c(new_n345), .d(new_n354), .o1(\s[31] ));
  xnbna2aa1n03x5               g260(.a(new_n107), .b(new_n132), .c(new_n98), .out0(\s[3] ));
  inv000aa1d42x5               g261(.a(new_n105), .o1(new_n357));
  nanp02aa1n02x5               g262(.a(new_n101), .b(new_n107), .o1(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n104), .b(new_n358), .c(new_n357), .out0(\s[4] ));
  xorc02aa1n02x5               g264(.a(\a[5] ), .b(\b[4] ), .out0(new_n360));
  xnbna2aa1n03x5               g265(.a(new_n360), .b(new_n108), .c(new_n109), .out0(\s[5] ));
  inv000aa1d42x5               g266(.a(\a[5] ), .o1(new_n362));
  inv000aa1d42x5               g267(.a(\b[4] ), .o1(new_n363));
  aobi12aa1n02x5               g268(.a(new_n360), .b(new_n108), .c(new_n109), .out0(new_n364));
  xorc02aa1n02x5               g269(.a(\a[6] ), .b(\b[5] ), .out0(new_n365));
  aoai13aa1n02x5               g270(.a(new_n365), .b(new_n364), .c(new_n362), .d(new_n363), .o1(new_n366));
  aoi112aa1n02x5               g271(.a(new_n364), .b(new_n365), .c(new_n362), .d(new_n363), .o1(new_n367));
  norb02aa1n02x5               g272(.a(new_n366), .b(new_n367), .out0(\s[6] ));
  aoai13aa1n02x5               g273(.a(new_n118), .b(new_n115), .c(new_n134), .d(new_n360), .o1(new_n369));
  inv000aa1d42x5               g274(.a(\b[5] ), .o1(new_n370));
  aboi22aa1n03x5               g275(.a(\a[6] ), .b(new_n370), .c(new_n119), .d(new_n111), .out0(new_n371));
  aobi12aa1n02x5               g276(.a(new_n369), .b(new_n366), .c(new_n371), .out0(\s[7] ));
  xnbna2aa1n03x5               g277(.a(new_n116), .b(new_n369), .c(new_n119), .out0(\s[8] ));
  nanp02aa1n02x5               g278(.a(new_n134), .b(new_n138), .o1(new_n374));
  aoi113aa1n02x5               g279(.a(new_n128), .b(new_n120), .c(new_n118), .d(new_n116), .e(new_n115), .o1(new_n375));
  aoi022aa1n02x5               g280(.a(new_n122), .b(new_n128), .c(new_n374), .d(new_n375), .o1(\s[9] ));
endmodule


