// Benchmark "adder" written by ABC on Thu Jul 18 04:31:48 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n355, new_n356, new_n357, new_n359, new_n361,
    new_n362, new_n363, new_n364, new_n366, new_n367, new_n369, new_n371,
    new_n372, new_n373;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  norb02aa1n03x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  nanp02aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai112aa1n06x5               g006(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n102));
  oai022aa1d24x5               g007(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  aoi013aa1n06x4               g008(.a(new_n103), .b(new_n100), .c(new_n101), .d(new_n102), .o1(new_n104));
  nand02aa1d10x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  oai012aa1n06x5               g010(.a(new_n105), .b(\b[5] ), .c(\a[6] ), .o1(new_n106));
  nanp02aa1n09x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor042aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nano22aa1n03x7               g014(.a(new_n108), .b(new_n107), .c(new_n109), .out0(new_n110));
  nand02aa1d16x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  oai012aa1n06x5               g016(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1d28x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norb02aa1n03x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nona23aa1n09x5               g020(.a(new_n110), .b(new_n115), .c(new_n112), .d(new_n106), .out0(new_n116));
  nanb03aa1n03x5               g021(.a(new_n113), .b(new_n114), .c(new_n109), .out0(new_n117));
  oai022aa1d18x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  oai112aa1n03x5               g023(.a(new_n118), .b(new_n107), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  inv020aa1n04x5               g024(.a(new_n113), .o1(new_n120));
  tech160nm_fioaoi03aa1n02p5x5 g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  oab012aa1n06x5               g026(.a(new_n121), .b(new_n119), .c(new_n117), .out0(new_n122));
  oai012aa1n12x5               g027(.a(new_n122), .b(new_n104), .c(new_n116), .o1(new_n123));
  nand42aa1n08x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n97), .b(new_n123), .c(new_n124), .o1(new_n125));
  xnrb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n08x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  oai012aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n129));
  nano23aa1n03x5               g034(.a(new_n97), .b(new_n127), .c(new_n128), .d(new_n124), .out0(new_n130));
  nanp02aa1n02x5               g035(.a(new_n123), .b(new_n130), .o1(new_n131));
  nor002aa1n20x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n09x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n131), .c(new_n129), .out0(\s[11] ));
  inv000aa1d42x5               g040(.a(new_n132), .o1(new_n136));
  aob012aa1n03x5               g041(.a(new_n134), .b(new_n131), .c(new_n129), .out0(new_n137));
  nor042aa1d18x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n24x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n03x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n136), .out0(\s[12] ));
  nona23aa1n02x4               g046(.a(new_n128), .b(new_n124), .c(new_n97), .d(new_n127), .out0(new_n142));
  nano22aa1n03x7               g047(.a(new_n142), .b(new_n134), .c(new_n140), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n123), .b(new_n143), .o1(new_n144));
  nanb02aa1n06x5               g049(.a(new_n98), .b(new_n99), .out0(new_n145));
  nor002aa1n02x5               g050(.a(\b[1] ), .b(\a[2] ), .o1(new_n146));
  nand22aa1n02x5               g051(.a(\b[0] ), .b(\a[1] ), .o1(new_n147));
  tech160nm_fioai012aa1n05x5   g052(.a(new_n101), .b(new_n146), .c(new_n147), .o1(new_n148));
  oabi12aa1n06x5               g053(.a(new_n103), .b(new_n148), .c(new_n145), .out0(new_n149));
  nano23aa1n02x5               g054(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n114), .o1(new_n151));
  nor003aa1n02x5               g056(.a(new_n112), .b(new_n113), .c(new_n151), .o1(new_n152));
  nand23aa1n04x5               g057(.a(new_n149), .b(new_n150), .c(new_n152), .o1(new_n153));
  nand23aa1n03x5               g058(.a(new_n130), .b(new_n134), .c(new_n140), .o1(new_n154));
  nanb03aa1n12x5               g059(.a(new_n138), .b(new_n139), .c(new_n133), .out0(new_n155));
  oai022aa1d24x5               g060(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n156));
  oai112aa1n06x5               g061(.a(new_n156), .b(new_n128), .c(\b[10] ), .d(\a[11] ), .o1(new_n157));
  aoi012aa1d24x5               g062(.a(new_n138), .b(new_n132), .c(new_n139), .o1(new_n158));
  oai012aa1d24x5               g063(.a(new_n158), .b(new_n157), .c(new_n155), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n154), .c(new_n153), .d(new_n122), .o1(new_n161));
  nor042aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand22aa1n06x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  nano22aa1n03x5               g069(.a(new_n138), .b(new_n133), .c(new_n139), .out0(new_n165));
  aoi012aa1n02x5               g070(.a(new_n132), .b(\a[10] ), .c(\b[9] ), .o1(new_n166));
  nand03aa1n02x5               g071(.a(new_n165), .b(new_n156), .c(new_n166), .o1(new_n167));
  nano22aa1n02x4               g072(.a(new_n164), .b(new_n167), .c(new_n158), .out0(new_n168));
  aoi022aa1n02x5               g073(.a(new_n161), .b(new_n164), .c(new_n144), .d(new_n168), .o1(\s[13] ));
  aoi012aa1n02x5               g074(.a(new_n162), .b(new_n161), .c(new_n163), .o1(new_n170));
  xnrb03aa1n02x5               g075(.a(new_n170), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n06x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand22aa1n06x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nano23aa1n06x5               g078(.a(new_n162), .b(new_n172), .c(new_n173), .d(new_n163), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n159), .c(new_n123), .d(new_n143), .o1(new_n175));
  oa0012aa1n02x5               g080(.a(new_n173), .b(new_n172), .c(new_n162), .o(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  nor042aa1d18x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand22aa1n03x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n03x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n175), .c(new_n177), .out0(\s[15] ));
  aobi12aa1n06x5               g086(.a(new_n180), .b(new_n175), .c(new_n177), .out0(new_n182));
  nor042aa1n06x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n06x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n06x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  oabi12aa1n02x5               g090(.a(new_n185), .b(new_n182), .c(new_n178), .out0(new_n186));
  inv040aa1n06x5               g091(.a(new_n178), .o1(new_n187));
  nano22aa1n03x7               g092(.a(new_n182), .b(new_n187), .c(new_n185), .out0(new_n188));
  nanb02aa1n02x5               g093(.a(new_n188), .b(new_n186), .out0(\s[16] ));
  nand23aa1n03x5               g094(.a(new_n174), .b(new_n180), .c(new_n185), .o1(new_n190));
  nor042aa1n04x5               g095(.a(new_n190), .b(new_n154), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n123), .b(new_n191), .o1(new_n192));
  xorc02aa1n12x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  nona23aa1n09x5               g098(.a(new_n173), .b(new_n163), .c(new_n162), .d(new_n172), .out0(new_n194));
  nano22aa1n03x7               g099(.a(new_n194), .b(new_n180), .c(new_n185), .out0(new_n195));
  nanb03aa1n03x5               g100(.a(new_n183), .b(new_n184), .c(new_n179), .out0(new_n196));
  oai112aa1n04x5               g101(.a(new_n187), .b(new_n173), .c(new_n172), .d(new_n162), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n193), .b(new_n183), .c(new_n184), .d(new_n178), .o1(new_n198));
  oai012aa1n02x5               g103(.a(new_n198), .b(new_n197), .c(new_n196), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n199), .b(new_n159), .c(new_n195), .o1(new_n200));
  nand42aa1n02x5               g105(.a(new_n143), .b(new_n195), .o1(new_n201));
  oaoi03aa1n03x5               g106(.a(\a[16] ), .b(\b[15] ), .c(new_n187), .o1(new_n202));
  oabi12aa1n03x5               g107(.a(new_n202), .b(new_n196), .c(new_n197), .out0(new_n203));
  tech160nm_fiaoi012aa1n05x5   g108(.a(new_n203), .b(new_n159), .c(new_n195), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n201), .c(new_n153), .d(new_n122), .o1(new_n205));
  aoi022aa1n02x5               g110(.a(new_n205), .b(new_n193), .c(new_n192), .d(new_n200), .o1(\s[17] ));
  inv000aa1d42x5               g111(.a(\a[17] ), .o1(new_n207));
  inv000aa1d42x5               g112(.a(\b[16] ), .o1(new_n208));
  nanp02aa1n02x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  oaib12aa1n06x5               g114(.a(new_n205), .b(new_n208), .c(\a[17] ), .out0(new_n210));
  xorc02aa1n06x5               g115(.a(\a[18] ), .b(\b[17] ), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n210), .c(new_n209), .out0(\s[18] ));
  oab012aa1n03x5               g117(.a(new_n202), .b(new_n197), .c(new_n196), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n190), .c(new_n167), .d(new_n158), .o1(new_n214));
  and002aa1n03x5               g119(.a(new_n211), .b(new_n193), .o(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n214), .c(new_n123), .d(new_n191), .o1(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n209), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  nor042aa1n12x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand02aa1d06x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  xnbna2aa1n03x5               g126(.a(new_n221), .b(new_n216), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g128(.a(new_n216), .b(new_n218), .o1(new_n224));
  nor002aa1n12x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nand02aa1d16x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n219), .c(new_n224), .d(new_n220), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n221), .b(new_n217), .c(new_n205), .d(new_n215), .o1(new_n229));
  nona22aa1n06x5               g134(.a(new_n229), .b(new_n227), .c(new_n219), .out0(new_n230));
  nanp02aa1n03x5               g135(.a(new_n228), .b(new_n230), .o1(\s[20] ));
  nano23aa1n09x5               g136(.a(new_n219), .b(new_n225), .c(new_n226), .d(new_n220), .out0(new_n232));
  nand23aa1n03x5               g137(.a(new_n232), .b(new_n193), .c(new_n211), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n214), .c(new_n123), .d(new_n191), .o1(new_n235));
  nanb03aa1n06x5               g140(.a(new_n225), .b(new_n226), .c(new_n220), .out0(new_n236));
  nand02aa1n04x5               g141(.a(\b[17] ), .b(\a[18] ), .o1(new_n237));
  oaih22aa1d12x5               g142(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n238));
  oai112aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(\b[18] ), .d(\a[19] ), .o1(new_n239));
  tech160nm_fiaoi012aa1n04x5   g144(.a(new_n225), .b(new_n219), .c(new_n226), .o1(new_n240));
  oai012aa1n04x7               g145(.a(new_n240), .b(new_n239), .c(new_n236), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n241), .b(new_n235), .out0(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[20] ), .b(\a[21] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nano22aa1n03x7               g149(.a(new_n225), .b(new_n220), .c(new_n226), .out0(new_n245));
  oai012aa1n02x5               g150(.a(new_n237), .b(\b[18] ), .c(\a[19] ), .o1(new_n246));
  norb02aa1n02x7               g151(.a(new_n238), .b(new_n246), .out0(new_n247));
  inv000aa1n02x5               g152(.a(new_n240), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n248), .b(new_n244), .c(new_n247), .d(new_n245), .o1(new_n249));
  aoi022aa1n02x5               g154(.a(new_n242), .b(new_n244), .c(new_n235), .d(new_n249), .o1(\s[21] ));
  nor042aa1n06x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  tech160nm_fixnrc02aa1n04x5   g156(.a(\b[21] ), .b(\a[22] ), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n251), .c(new_n242), .d(new_n244), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n244), .b(new_n241), .c(new_n205), .d(new_n234), .o1(new_n254));
  nona22aa1n02x4               g159(.a(new_n254), .b(new_n252), .c(new_n251), .out0(new_n255));
  nanp02aa1n02x5               g160(.a(new_n253), .b(new_n255), .o1(\s[22] ));
  nor042aa1n06x5               g161(.a(new_n252), .b(new_n243), .o1(new_n257));
  nand23aa1n03x5               g162(.a(new_n215), .b(new_n257), .c(new_n232), .o1(new_n258));
  inv000aa1n06x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n214), .c(new_n123), .d(new_n191), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\a[22] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(\b[21] ), .o1(new_n262));
  oaoi03aa1n12x5               g167(.a(new_n261), .b(new_n262), .c(new_n251), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  aoi012aa1n02x5               g169(.a(new_n264), .b(new_n241), .c(new_n257), .o1(new_n265));
  nand42aa1n04x5               g170(.a(new_n260), .b(new_n265), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[23] ), .b(\b[22] ), .out0(new_n267));
  aoi112aa1n02x5               g172(.a(new_n267), .b(new_n264), .c(new_n241), .d(new_n257), .o1(new_n268));
  aoi022aa1n02x5               g173(.a(new_n266), .b(new_n267), .c(new_n260), .d(new_n268), .o1(\s[23] ));
  norp02aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .o1(new_n270));
  xnrc02aa1n12x5               g175(.a(\b[23] ), .b(\a[24] ), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n266), .d(new_n267), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n266), .b(new_n267), .o1(new_n273));
  nona22aa1n02x4               g178(.a(new_n273), .b(new_n271), .c(new_n270), .out0(new_n274));
  nanp02aa1n03x5               g179(.a(new_n274), .b(new_n272), .o1(\s[24] ));
  aoai13aa1n06x5               g180(.a(new_n257), .b(new_n248), .c(new_n247), .d(new_n245), .o1(new_n276));
  norb02aa1n06x5               g181(.a(new_n267), .b(new_n271), .out0(new_n277));
  inv040aa1n02x5               g182(.a(new_n277), .o1(new_n278));
  oai022aa1n02x5               g183(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n279));
  aob012aa1n02x5               g184(.a(new_n279), .b(\b[23] ), .c(\a[24] ), .out0(new_n280));
  aoai13aa1n12x5               g185(.a(new_n280), .b(new_n278), .c(new_n276), .d(new_n263), .o1(new_n281));
  inv000aa1n02x5               g186(.a(new_n281), .o1(new_n282));
  nona22aa1n03x5               g187(.a(new_n205), .b(new_n258), .c(new_n278), .out0(new_n283));
  nanp02aa1n06x5               g188(.a(new_n283), .b(new_n282), .o1(new_n284));
  tech160nm_fixorc02aa1n03p5x5 g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n277), .b(new_n264), .c(new_n241), .d(new_n257), .o1(new_n286));
  aoi012aa1n12x5               g191(.a(new_n214), .b(new_n123), .c(new_n191), .o1(new_n287));
  nano22aa1n03x7               g192(.a(new_n287), .b(new_n259), .c(new_n277), .out0(new_n288));
  nano23aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n280), .d(new_n286), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(new_n284), .c(new_n285), .o1(\s[25] ));
  norp02aa1n02x5               g195(.a(\b[24] ), .b(\a[25] ), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[25] ), .b(\a[26] ), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n291), .c(new_n284), .d(new_n285), .o1(new_n293));
  tech160nm_fioai012aa1n05x5   g198(.a(new_n285), .b(new_n288), .c(new_n281), .o1(new_n294));
  nona22aa1n03x5               g199(.a(new_n294), .b(new_n292), .c(new_n291), .out0(new_n295));
  nanp02aa1n03x5               g200(.a(new_n293), .b(new_n295), .o1(\s[26] ));
  nanb02aa1n09x5               g201(.a(new_n292), .b(new_n285), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  nano32aa1d12x5               g203(.a(new_n287), .b(new_n298), .c(new_n259), .d(new_n277), .out0(new_n299));
  nor042aa1n06x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  and002aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o(new_n301));
  norp02aa1n02x5               g206(.a(new_n301), .b(new_n300), .o1(new_n302));
  aoi112aa1n02x5               g207(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n303));
  inv000aa1d42x5               g208(.a(\a[26] ), .o1(new_n304));
  inv000aa1d42x5               g209(.a(\b[25] ), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n300), .o1(new_n306));
  aboi22aa1n03x5               g211(.a(new_n301), .b(new_n306), .c(new_n305), .d(new_n304), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n297), .c(new_n286), .d(new_n280), .o1(new_n308));
  nor003aa1n03x5               g213(.a(new_n299), .b(new_n308), .c(new_n303), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n303), .b(new_n304), .c(new_n305), .o1(new_n310));
  aoai13aa1n04x5               g215(.a(new_n310), .b(new_n297), .c(new_n286), .d(new_n280), .o1(new_n311));
  oaoi13aa1n03x5               g216(.a(new_n309), .b(new_n302), .c(new_n311), .d(new_n299), .o1(\s[27] ));
  oabi12aa1n03x5               g217(.a(new_n301), .b(new_n299), .c(new_n311), .out0(new_n313));
  nona32aa1n03x5               g218(.a(new_n205), .b(new_n297), .c(new_n278), .d(new_n258), .out0(new_n314));
  aobi12aa1n12x5               g219(.a(new_n310), .b(new_n281), .c(new_n298), .out0(new_n315));
  aoai13aa1n02x7               g220(.a(new_n306), .b(new_n301), .c(new_n315), .d(new_n314), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .out0(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n300), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[28] ));
  inv000aa1d42x5               g224(.a(\a[27] ), .o1(new_n320));
  inv000aa1d42x5               g225(.a(\a[28] ), .o1(new_n321));
  xroi22aa1d06x4               g226(.a(new_n320), .b(\b[26] ), .c(new_n321), .d(\b[27] ), .out0(new_n322));
  oai012aa1n03x5               g227(.a(new_n322), .b(new_n299), .c(new_n311), .o1(new_n323));
  inv000aa1n06x5               g228(.a(new_n322), .o1(new_n324));
  oao003aa1n02x5               g229(.a(\a[28] ), .b(\b[27] ), .c(new_n306), .carry(new_n325));
  aoai13aa1n02x7               g230(.a(new_n325), .b(new_n324), .c(new_n315), .d(new_n314), .o1(new_n326));
  xorc02aa1n02x5               g231(.a(\a[29] ), .b(\b[28] ), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n325), .b(new_n327), .out0(new_n328));
  aoi022aa1n03x5               g233(.a(new_n326), .b(new_n327), .c(new_n323), .d(new_n328), .o1(\s[29] ));
  xorb03aa1n02x5               g234(.a(new_n147), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g235(.a(new_n317), .b(new_n327), .c(new_n302), .o(new_n331));
  oai012aa1n03x5               g236(.a(new_n331), .b(new_n299), .c(new_n311), .o1(new_n332));
  inv000aa1n02x5               g237(.a(new_n331), .o1(new_n333));
  inv000aa1d42x5               g238(.a(\b[28] ), .o1(new_n334));
  inv000aa1d42x5               g239(.a(\a[29] ), .o1(new_n335));
  oaib12aa1n02x5               g240(.a(new_n325), .b(\b[28] ), .c(new_n335), .out0(new_n336));
  oaib12aa1n02x5               g241(.a(new_n336), .b(new_n334), .c(\a[29] ), .out0(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n333), .c(new_n315), .d(new_n314), .o1(new_n338));
  xorc02aa1n02x5               g243(.a(\a[30] ), .b(\b[29] ), .out0(new_n339));
  oaoi13aa1n02x5               g244(.a(new_n339), .b(new_n336), .c(new_n335), .d(new_n334), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n338), .b(new_n339), .c(new_n332), .d(new_n340), .o1(\s[30] ));
  nano22aa1n02x4               g246(.a(new_n324), .b(new_n327), .c(new_n339), .out0(new_n342));
  oai012aa1n03x5               g247(.a(new_n342), .b(new_n299), .c(new_n311), .o1(new_n343));
  aoi022aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n344));
  norb02aa1n02x5               g249(.a(\a[31] ), .b(\b[30] ), .out0(new_n345));
  obai22aa1n02x7               g250(.a(\b[30] ), .b(\a[31] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n346));
  aoi112aa1n02x5               g251(.a(new_n346), .b(new_n345), .c(new_n336), .d(new_n344), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[31] ), .b(\b[30] ), .out0(new_n348));
  inv000aa1n02x5               g253(.a(new_n342), .o1(new_n349));
  norp02aa1n02x5               g254(.a(\b[29] ), .b(\a[30] ), .o1(new_n350));
  aoi012aa1n02x5               g255(.a(new_n350), .b(new_n336), .c(new_n344), .o1(new_n351));
  aoai13aa1n02x7               g256(.a(new_n351), .b(new_n349), .c(new_n315), .d(new_n314), .o1(new_n352));
  aoi022aa1n03x5               g257(.a(new_n352), .b(new_n348), .c(new_n343), .d(new_n347), .o1(\s[31] ));
  xnbna2aa1n03x5               g258(.a(new_n145), .b(new_n102), .c(new_n101), .out0(\s[3] ));
  oaoi03aa1n02x5               g259(.a(\a[3] ), .b(\b[2] ), .c(new_n148), .o1(new_n355));
  xorc02aa1n02x5               g260(.a(\a[4] ), .b(\b[3] ), .out0(new_n356));
  aoi113aa1n02x5               g261(.a(new_n356), .b(new_n98), .c(new_n102), .d(new_n99), .e(new_n101), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n357), .b(new_n355), .c(new_n356), .o1(\s[4] ));
  xnrc02aa1n02x5               g263(.a(\b[4] ), .b(\a[5] ), .out0(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n359), .b(new_n149), .c(new_n111), .out0(\s[5] ));
  tech160nm_fiaoi012aa1n05x5   g265(.a(new_n359), .b(new_n149), .c(new_n111), .o1(new_n361));
  xorc02aa1n02x5               g266(.a(\a[6] ), .b(\b[5] ), .out0(new_n362));
  aoai13aa1n06x5               g267(.a(new_n362), .b(new_n361), .c(\a[5] ), .d(\b[4] ), .o1(new_n363));
  norb02aa1n02x5               g268(.a(new_n105), .b(new_n362), .out0(new_n364));
  oaib12aa1n02x5               g269(.a(new_n363), .b(new_n361), .c(new_n364), .out0(\s[6] ));
  aoi022aa1n02x5               g270(.a(new_n363), .b(new_n107), .c(new_n120), .d(new_n114), .o1(new_n366));
  nona23aa1n03x5               g271(.a(new_n363), .b(new_n107), .c(new_n113), .d(new_n151), .out0(new_n367));
  norb02aa1n02x5               g272(.a(new_n367), .b(new_n366), .out0(\s[7] ));
  norb02aa1n02x5               g273(.a(new_n109), .b(new_n108), .out0(new_n369));
  xnbna2aa1n03x5               g274(.a(new_n369), .b(new_n367), .c(new_n120), .out0(\s[8] ));
  norb02aa1n02x5               g275(.a(new_n124), .b(new_n97), .out0(new_n371));
  norp02aa1n02x5               g276(.a(new_n119), .b(new_n117), .o1(new_n372));
  norp03aa1n02x5               g277(.a(new_n372), .b(new_n121), .c(new_n371), .o1(new_n373));
  aoi022aa1n02x5               g278(.a(new_n123), .b(new_n371), .c(new_n153), .d(new_n373), .o1(\s[9] ));
endmodule


