// Benchmark "adder" written by ABC on Thu Jul 18 06:28:39 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n336, new_n337,
    new_n338, new_n339, new_n340, new_n342, new_n344, new_n345, new_n346,
    new_n348, new_n350, new_n351, new_n352, new_n353, new_n355, new_n357;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n06x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fioaoi03aa1n03p5x5 g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor002aa1d32x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n04x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fioai012aa1n03p5x5 g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n06x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nanp02aa1n12x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand02aa1n06x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n110), .b(new_n113), .c(new_n112), .d(new_n111), .out0(new_n114));
  xnrc02aa1n12x5               g019(.a(\b[7] ), .b(\a[8] ), .out0(new_n115));
  nor042aa1d18x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand02aa1n06x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nanb02aa1n03x5               g022(.a(new_n116), .b(new_n117), .out0(new_n118));
  nor043aa1n03x5               g023(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n119));
  nanb03aa1n06x5               g024(.a(new_n116), .b(new_n117), .c(new_n110), .out0(new_n120));
  nor042aa1n02x5               g025(.a(new_n112), .b(new_n111), .o1(new_n121));
  aoi112aa1n03x5               g026(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n122));
  oab012aa1n04x5               g027(.a(new_n122), .b(\a[8] ), .c(\b[7] ), .out0(new_n123));
  oai013aa1n06x5               g028(.a(new_n123), .b(new_n120), .c(new_n115), .d(new_n121), .o1(new_n124));
  nand42aa1n03x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n97), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n127));
  nor002aa1n10x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  norp02aa1n02x5               g035(.a(new_n128), .b(new_n97), .o1(new_n131));
  nanp03aa1n02x5               g036(.a(new_n127), .b(new_n129), .c(new_n131), .o1(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n130), .c(new_n98), .d(new_n127), .o1(\s[10] ));
  nano23aa1n03x7               g038(.a(new_n97), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n135));
  oai012aa1n02x5               g040(.a(new_n129), .b(new_n128), .c(new_n97), .o1(new_n136));
  nor042aa1n12x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand02aa1d10x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n135), .c(new_n136), .out0(\s[11] ));
  inv030aa1n03x5               g045(.a(new_n137), .o1(new_n141));
  aob012aa1n02x5               g046(.a(new_n139), .b(new_n135), .c(new_n136), .out0(new_n142));
  nor042aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand22aa1n12x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n142), .b(new_n144), .c(new_n143), .d(new_n137), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n141), .d(new_n142), .o1(\s[12] ));
  nano23aa1n03x7               g052(.a(new_n137), .b(new_n143), .c(new_n144), .d(new_n138), .out0(new_n148));
  and002aa1n02x5               g053(.a(new_n148), .b(new_n134), .o(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n109), .b(new_n119), .o1(new_n151));
  norp03aa1n02x5               g056(.a(new_n120), .b(new_n115), .c(new_n121), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n123), .b(new_n152), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n151), .b(new_n153), .o1(new_n154));
  tech160nm_fioai012aa1n03p5x5 g059(.a(new_n144), .b(new_n143), .c(new_n137), .o1(new_n155));
  nano22aa1n03x7               g060(.a(new_n143), .b(new_n138), .c(new_n144), .out0(new_n156));
  oai112aa1n06x5               g061(.a(new_n141), .b(new_n129), .c(new_n128), .d(new_n97), .o1(new_n157));
  oaib12aa1n18x5               g062(.a(new_n155), .b(new_n157), .c(new_n156), .out0(new_n158));
  nor002aa1d32x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n04x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n158), .c(new_n154), .d(new_n149), .o1(new_n162));
  oaib12aa1n02x5               g067(.a(new_n155), .b(new_n159), .c(new_n160), .out0(new_n163));
  aoib12aa1n02x5               g068(.a(new_n163), .b(new_n156), .c(new_n157), .out0(new_n164));
  aobi12aa1n02x5               g069(.a(new_n162), .b(new_n164), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g070(.a(new_n159), .o1(new_n166));
  nor022aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  oaih22aa1n06x5               g074(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n170));
  nanb03aa1n02x5               g075(.a(new_n170), .b(new_n162), .c(new_n168), .out0(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n169), .c(new_n166), .d(new_n162), .o1(\s[14] ));
  nano23aa1n06x5               g077(.a(new_n159), .b(new_n167), .c(new_n168), .d(new_n160), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n158), .c(new_n154), .d(new_n149), .o1(new_n174));
  oai012aa1n02x5               g079(.a(new_n168), .b(new_n167), .c(new_n159), .o1(new_n175));
  nor042aa1d18x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanp02aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  xobna2aa1n03x5               g083(.a(new_n178), .b(new_n174), .c(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g084(.a(new_n158), .o1(new_n180));
  nona23aa1n03x5               g085(.a(new_n168), .b(new_n160), .c(new_n159), .d(new_n167), .out0(new_n181));
  aoai13aa1n02x5               g086(.a(new_n175), .b(new_n181), .c(new_n150), .d(new_n180), .o1(new_n182));
  nor042aa1n06x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nanp02aa1n04x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n183), .b(new_n184), .out0(new_n185));
  aoai13aa1n02x5               g090(.a(new_n185), .b(new_n176), .c(new_n182), .d(new_n177), .o1(new_n186));
  norp02aa1n02x5               g091(.a(new_n183), .b(new_n176), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n187), .o1(new_n188));
  aoai13aa1n02x5               g093(.a(new_n184), .b(new_n178), .c(new_n174), .d(new_n175), .o1(new_n189));
  oai012aa1n02x5               g094(.a(new_n186), .b(new_n189), .c(new_n188), .o1(\s[16] ));
  nano23aa1n06x5               g095(.a(new_n176), .b(new_n183), .c(new_n184), .d(new_n177), .out0(new_n191));
  nand02aa1d04x5               g096(.a(new_n191), .b(new_n173), .o1(new_n192));
  nano22aa1n03x7               g097(.a(new_n192), .b(new_n134), .c(new_n148), .out0(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n194));
  nor043aa1n03x5               g099(.a(new_n181), .b(new_n178), .c(new_n185), .o1(new_n195));
  oai012aa1n02x5               g100(.a(new_n184), .b(new_n183), .c(new_n176), .o1(new_n196));
  nanb03aa1n02x5               g101(.a(new_n183), .b(new_n184), .c(new_n177), .out0(new_n197));
  oai112aa1n02x5               g102(.a(new_n170), .b(new_n168), .c(\b[14] ), .d(\a[15] ), .o1(new_n198));
  tech160nm_fioai012aa1n04x5   g103(.a(new_n196), .b(new_n198), .c(new_n197), .o1(new_n199));
  aoi012aa1d18x5               g104(.a(new_n199), .b(new_n158), .c(new_n195), .o1(new_n200));
  nanp02aa1n12x5               g105(.a(new_n194), .b(new_n200), .o1(new_n201));
  nor022aa1n16x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  nand42aa1n06x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nanb02aa1n12x5               g109(.a(new_n202), .b(new_n203), .out0(new_n205));
  oai112aa1n02x5               g110(.a(new_n196), .b(new_n205), .c(new_n198), .d(new_n197), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n206), .b(new_n158), .c(new_n195), .o1(new_n207));
  aoi022aa1n02x5               g112(.a(new_n201), .b(new_n204), .c(new_n207), .d(new_n194), .o1(\s[17] ));
  aoi012aa1n02x5               g113(.a(new_n202), .b(new_n201), .c(new_n203), .o1(new_n209));
  nor042aa1n02x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nanp02aa1n04x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  norb02aa1n06x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  oai022aa1d24x5               g117(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n213));
  aoi122aa1n06x5               g118(.a(new_n213), .b(\b[17] ), .c(\a[18] ), .d(new_n201), .e(new_n204), .o1(new_n214));
  oabi12aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n212), .out0(\s[18] ));
  nano23aa1n02x4               g120(.a(new_n202), .b(new_n210), .c(new_n211), .d(new_n203), .out0(new_n216));
  nand22aa1n03x5               g121(.a(new_n201), .b(new_n216), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n211), .b(new_n210), .c(new_n202), .o1(new_n218));
  nor042aa1n04x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand42aa1n08x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  norb02aa1n15x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  xnbna2aa1n03x5               g126(.a(new_n221), .b(new_n217), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n02x5               g128(.a(new_n216), .o1(new_n224));
  aoai13aa1n02x5               g129(.a(new_n218), .b(new_n224), .c(new_n194), .d(new_n200), .o1(new_n225));
  orn002aa1n12x5               g130(.a(\a[20] ), .b(\b[19] ), .o(new_n226));
  nanp02aa1n06x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand22aa1n03x5               g132(.a(new_n226), .b(new_n227), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n219), .c(new_n225), .d(new_n220), .o1(new_n229));
  oai022aa1n02x5               g134(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n221), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n227), .b(new_n231), .c(new_n217), .d(new_n218), .o1(new_n232));
  tech160nm_fioai012aa1n02p5x5 g137(.a(new_n229), .b(new_n232), .c(new_n230), .o1(\s[20] ));
  nona23aa1d18x5               g138(.a(new_n221), .b(new_n212), .c(new_n228), .d(new_n205), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nanp03aa1n06x5               g140(.a(new_n226), .b(new_n220), .c(new_n227), .o1(new_n236));
  oai112aa1n06x5               g141(.a(new_n213), .b(new_n211), .c(\b[18] ), .d(\a[19] ), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n230), .b(new_n227), .o1(new_n238));
  oai012aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(new_n236), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[21] ), .b(\b[20] ), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n239), .c(new_n201), .d(new_n235), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n237), .b(new_n236), .o1(new_n242));
  nona22aa1n02x4               g147(.a(new_n238), .b(new_n242), .c(new_n240), .out0(new_n243));
  aoi012aa1n02x5               g148(.a(new_n243), .b(new_n201), .c(new_n235), .o1(new_n244));
  norb02aa1n03x4               g149(.a(new_n241), .b(new_n244), .out0(\s[21] ));
  inv000aa1d42x5               g150(.a(\a[21] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(\b[20] ), .b(new_n246), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .out0(new_n248));
  and002aa1n02x5               g153(.a(\b[21] ), .b(\a[22] ), .o(new_n249));
  oai022aa1n02x5               g154(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n250));
  nona22aa1n02x5               g155(.a(new_n241), .b(new_n249), .c(new_n250), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n248), .c(new_n247), .d(new_n241), .o1(\s[22] ));
  inv020aa1n04x5               g157(.a(\a[22] ), .o1(new_n253));
  xroi22aa1d06x4               g158(.a(new_n246), .b(\b[20] ), .c(new_n253), .d(\b[21] ), .out0(new_n254));
  norb02aa1n02x7               g159(.a(new_n254), .b(new_n234), .out0(new_n255));
  oaoi03aa1n02x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .o1(new_n256));
  tech160nm_fiao0012aa1n02p5x5 g161(.a(new_n256), .b(new_n239), .c(new_n254), .o(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n257), .c(new_n201), .d(new_n255), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n258), .b(new_n256), .c(new_n239), .d(new_n254), .o1(new_n260));
  aobi12aa1n02x5               g165(.a(new_n260), .b(new_n201), .c(new_n255), .out0(new_n261));
  norb02aa1n03x4               g166(.a(new_n259), .b(new_n261), .out0(\s[23] ));
  norp02aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  tech160nm_fixorc02aa1n02p5x5 g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  and002aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o(new_n266));
  oai022aa1n02x5               g171(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n267));
  nona22aa1n02x5               g172(.a(new_n259), .b(new_n266), .c(new_n267), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n265), .c(new_n264), .d(new_n259), .o1(\s[24] ));
  nano32aa1n06x5               g174(.a(new_n234), .b(new_n265), .c(new_n254), .d(new_n258), .out0(new_n270));
  aob012aa1n02x5               g175(.a(new_n267), .b(\b[23] ), .c(\a[24] ), .out0(new_n271));
  and002aa1n02x7               g176(.a(new_n265), .b(new_n258), .o(new_n272));
  aoai13aa1n04x5               g177(.a(new_n272), .b(new_n256), .c(new_n239), .d(new_n254), .o1(new_n273));
  nand42aa1n03x5               g178(.a(new_n273), .b(new_n271), .o1(new_n274));
  xorc02aa1n12x5               g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n274), .c(new_n201), .d(new_n270), .o1(new_n276));
  nanb02aa1n02x5               g181(.a(new_n275), .b(new_n271), .out0(new_n277));
  aoi122aa1n02x7               g182(.a(new_n277), .b(new_n257), .c(new_n272), .d(new_n201), .e(new_n270), .o1(new_n278));
  norb02aa1n02x7               g183(.a(new_n276), .b(new_n278), .out0(\s[25] ));
  nor042aa1n03x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  and002aa1n02x5               g187(.a(\b[25] ), .b(\a[26] ), .o(new_n283));
  oai022aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n284));
  nona22aa1n02x5               g189(.a(new_n276), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n281), .d(new_n276), .o1(\s[26] ));
  nanp02aa1n02x5               g191(.a(new_n282), .b(new_n275), .o1(new_n287));
  nona23aa1d18x5               g192(.a(new_n254), .b(new_n272), .c(new_n234), .d(new_n287), .out0(new_n288));
  aoi012aa1n12x5               g193(.a(new_n288), .b(new_n194), .c(new_n200), .o1(new_n289));
  tech160nm_fioaoi03aa1n02p5x5 g194(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  aoai13aa1n12x5               g196(.a(new_n291), .b(new_n287), .c(new_n273), .d(new_n271), .o1(new_n292));
  xorc02aa1n02x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  oa0012aa1n06x5               g198(.a(new_n293), .b(new_n292), .c(new_n289), .o(new_n294));
  aoi113aa1n02x5               g199(.a(new_n290), .b(new_n293), .c(new_n274), .d(new_n275), .e(new_n282), .o1(new_n295));
  aoib12aa1n02x7               g200(.a(new_n294), .b(new_n295), .c(new_n289), .out0(\s[27] ));
  norp02aa1n02x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  tech160nm_fioai012aa1n04x5   g203(.a(new_n293), .b(new_n292), .c(new_n289), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[28] ), .b(\b[27] ), .out0(new_n300));
  oai022aa1n02x5               g205(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(\a[28] ), .c(\b[27] ), .o1(new_n302));
  tech160nm_finand02aa1n03p5x5 g207(.a(new_n299), .b(new_n302), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n299), .d(new_n298), .o1(\s[28] ));
  xorc02aa1n12x5               g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  and002aa1n02x5               g210(.a(new_n300), .b(new_n293), .o(new_n306));
  aob012aa1n09x5               g211(.a(new_n301), .b(\b[27] ), .c(\a[28] ), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n307), .o1(new_n308));
  oaoi13aa1n03x5               g213(.a(new_n308), .b(new_n306), .c(new_n292), .d(new_n289), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n305), .o1(new_n310));
  oaih12aa1n02x5               g215(.a(new_n306), .b(new_n292), .c(new_n289), .o1(new_n311));
  nona22aa1n02x5               g216(.a(new_n311), .b(new_n308), .c(new_n310), .out0(new_n312));
  oaih12aa1n02x5               g217(.a(new_n312), .b(new_n309), .c(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g218(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g219(.a(new_n310), .b(new_n293), .c(new_n300), .out0(new_n315));
  oaoi03aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .c(new_n307), .o1(new_n316));
  oaoi13aa1n03x5               g221(.a(new_n316), .b(new_n315), .c(new_n292), .d(new_n289), .o1(new_n317));
  norp02aa1n02x5               g222(.a(\b[29] ), .b(\a[30] ), .o1(new_n318));
  nanp02aa1n02x5               g223(.a(\b[29] ), .b(\a[30] ), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n319), .b(new_n318), .out0(new_n320));
  oaih12aa1n02x5               g225(.a(new_n315), .b(new_n292), .c(new_n289), .o1(new_n321));
  and002aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .o(new_n322));
  nona32aa1n02x5               g227(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n316), .out0(new_n323));
  oaih12aa1n02x5               g228(.a(new_n323), .b(new_n317), .c(new_n320), .o1(\s[30] ));
  nano32aa1n02x4               g229(.a(new_n310), .b(new_n300), .c(new_n293), .d(new_n320), .out0(new_n325));
  oaih12aa1n02x5               g230(.a(new_n325), .b(new_n292), .c(new_n289), .o1(new_n326));
  oai012aa1n02x5               g231(.a(new_n307), .b(\b[28] ), .c(\a[29] ), .o1(new_n327));
  nanp02aa1n02x5               g232(.a(\b[28] ), .b(\a[29] ), .o1(new_n328));
  nano22aa1n02x4               g233(.a(new_n318), .b(new_n328), .c(new_n319), .out0(new_n329));
  aoi012aa1n02x5               g234(.a(new_n318), .b(new_n327), .c(new_n329), .o1(new_n330));
  xorc02aa1n02x5               g235(.a(\a[31] ), .b(\b[30] ), .out0(new_n331));
  oai022aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(\b[30] ), .d(\a[31] ), .o1(new_n332));
  aoi122aa1n06x5               g237(.a(new_n332), .b(\b[30] ), .c(\a[31] ), .d(new_n327), .e(new_n329), .o1(new_n333));
  tech160nm_finand02aa1n03p5x5 g238(.a(new_n326), .b(new_n333), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n331), .c(new_n326), .d(new_n330), .o1(\s[31] ));
  aoi022aa1n02x5               g240(.a(new_n100), .b(new_n99), .c(\a[1] ), .d(\b[0] ), .o1(new_n336));
  oaib12aa1n02x5               g241(.a(new_n336), .b(new_n100), .c(\a[2] ), .out0(new_n337));
  oao003aa1n02x5               g242(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n338));
  norb02aa1n02x5               g243(.a(new_n106), .b(new_n105), .out0(new_n339));
  aboi22aa1n03x5               g244(.a(new_n105), .b(new_n106), .c(new_n99), .d(new_n100), .out0(new_n340));
  aoi022aa1n02x5               g245(.a(new_n338), .b(new_n339), .c(new_n337), .d(new_n340), .o1(\s[3] ));
  oaoi03aa1n02x5               g246(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nano23aa1n02x4               g248(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n344));
  nanp02aa1n02x5               g249(.a(new_n344), .b(new_n338), .o1(new_n345));
  norb02aa1n02x5               g250(.a(new_n113), .b(new_n112), .out0(new_n346));
  xnbna2aa1n03x5               g251(.a(new_n346), .b(new_n345), .c(new_n108), .out0(\s[5] ));
  aoi012aa1n02x5               g252(.a(new_n112), .b(new_n109), .c(new_n113), .o1(new_n348));
  xnrb03aa1n02x5               g253(.a(new_n348), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aobi12aa1n02x5               g254(.a(new_n108), .b(new_n344), .c(new_n338), .out0(new_n350));
  oai012aa1n02x5               g255(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n351));
  oaoi13aa1n02x5               g256(.a(new_n118), .b(new_n351), .c(new_n350), .d(new_n114), .o1(new_n352));
  oai112aa1n02x5               g257(.a(new_n118), .b(new_n351), .c(new_n350), .d(new_n114), .o1(new_n353));
  norb02aa1n02x5               g258(.a(new_n353), .b(new_n352), .out0(\s[7] ));
  norp02aa1n02x5               g259(.a(new_n352), .b(new_n116), .o1(new_n355));
  xnrb03aa1n02x5               g260(.a(new_n355), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  norb03aa1n02x5               g261(.a(new_n123), .b(new_n152), .c(new_n126), .out0(new_n357));
  aoi022aa1n02x5               g262(.a(new_n154), .b(new_n126), .c(new_n151), .d(new_n357), .o1(\s[9] ));
endmodule


