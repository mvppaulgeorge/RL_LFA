// Benchmark "adder" written by ABC on Wed Jul 17 17:20:53 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n350, new_n352,
    new_n353, new_n355, new_n356, new_n358, new_n359, new_n360, new_n362,
    new_n363, new_n365, new_n366, new_n368, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[4] ), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\b[3] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  tech160nm_fioaoi03aa1n03p5x5 g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor042aa1n09x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand42aa1n16x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nano32aa1n03x7               g009(.a(new_n103), .b(new_n104), .c(\a[1] ), .d(\b[0] ), .out0(new_n105));
  tech160nm_finand02aa1n05x5   g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  oaih12aa1n02x5               g011(.a(new_n106), .b(\b[3] ), .c(\a[4] ), .o1(new_n107));
  oai122aa1n06x5               g012(.a(new_n104), .b(new_n99), .c(new_n100), .d(\a[3] ), .e(\b[2] ), .o1(new_n108));
  oai013aa1n06x5               g013(.a(new_n102), .b(new_n105), .c(new_n108), .d(new_n107), .o1(new_n109));
  oaih22aa1d12x5               g014(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(new_n110), .o1(new_n111));
  aoi022aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nand42aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  oai012aa1n02x5               g019(.a(new_n114), .b(\b[6] ), .c(\a[7] ), .o1(new_n115));
  nano23aa1n06x5               g020(.a(new_n115), .b(new_n113), .c(new_n111), .d(new_n112), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor042aa1n06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanb03aa1n09x5               g023(.a(new_n118), .b(new_n114), .c(new_n117), .out0(new_n119));
  inv000aa1n03x5               g024(.a(new_n118), .o1(new_n120));
  tech160nm_fioaoi03aa1n03p5x5 g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  inv000aa1n02x5               g026(.a(new_n121), .o1(new_n122));
  oai013aa1n09x5               g027(.a(new_n122), .b(new_n119), .c(new_n113), .d(new_n111), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .d(new_n116), .o1(new_n125));
  nor042aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  tech160nm_finand02aa1n03p5x5 g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n12x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n103), .o1(new_n130));
  and002aa1n02x5               g035(.a(\b[0] ), .b(\a[1] ), .o(new_n131));
  nanp03aa1n02x5               g036(.a(new_n130), .b(new_n131), .c(new_n104), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n132), .b(new_n108), .c(new_n107), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n113), .o1(new_n134));
  nona23aa1n02x4               g039(.a(new_n134), .b(new_n112), .c(new_n115), .d(new_n110), .out0(new_n135));
  norp03aa1n02x5               g040(.a(new_n119), .b(new_n113), .c(new_n111), .o1(new_n136));
  nor002aa1n02x5               g041(.a(new_n136), .b(new_n121), .o1(new_n137));
  aoai13aa1n06x5               g042(.a(new_n137), .b(new_n135), .c(new_n133), .d(new_n102), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n128), .b(new_n97), .c(new_n138), .d(new_n124), .o1(new_n139));
  tech160nm_fiaoi012aa1n04x5   g044(.a(new_n126), .b(new_n97), .c(new_n127), .o1(new_n140));
  nand42aa1d28x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nor042aa1n09x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xobna2aa1n03x5               g048(.a(new_n143), .b(new_n139), .c(new_n140), .out0(\s[11] ));
  inv000aa1d42x5               g049(.a(new_n141), .o1(new_n145));
  norp02aa1n04x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n08x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  inv040aa1n02x5               g053(.a(new_n140), .o1(new_n149));
  nona22aa1n02x4               g054(.a(new_n139), .b(new_n149), .c(new_n142), .out0(new_n150));
  nona22aa1n02x4               g055(.a(new_n150), .b(new_n148), .c(new_n145), .out0(new_n151));
  aobi12aa1n02x5               g056(.a(new_n128), .b(new_n125), .c(new_n98), .out0(new_n152));
  inv000aa1n02x5               g057(.a(new_n142), .o1(new_n153));
  nano22aa1n02x4               g058(.a(new_n152), .b(new_n140), .c(new_n153), .out0(new_n154));
  oai012aa1n02x5               g059(.a(new_n148), .b(new_n154), .c(new_n145), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n151), .o1(\s[12] ));
  nano23aa1d12x5               g061(.a(new_n146), .b(new_n142), .c(new_n147), .d(new_n141), .out0(new_n157));
  nand23aa1d12x5               g062(.a(new_n157), .b(new_n124), .c(new_n128), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n123), .c(new_n109), .d(new_n116), .o1(new_n160));
  oaoi03aa1n09x5               g065(.a(\a[12] ), .b(\b[11] ), .c(new_n153), .o1(new_n161));
  aoi012aa1n06x5               g066(.a(new_n161), .b(new_n157), .c(new_n149), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(new_n160), .b(new_n162), .o1(new_n163));
  nand42aa1d28x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nor042aa1n04x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  aoi112aa1n02x5               g071(.a(new_n161), .b(new_n166), .c(new_n157), .d(new_n149), .o1(new_n167));
  aoi022aa1n02x5               g072(.a(new_n163), .b(new_n166), .c(new_n160), .d(new_n167), .o1(\s[13] ));
  aoi012aa1n02x5               g073(.a(new_n165), .b(new_n163), .c(new_n164), .o1(new_n169));
  xnrb03aa1n02x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n10x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nano23aa1d15x5               g077(.a(new_n171), .b(new_n165), .c(new_n172), .d(new_n164), .out0(new_n173));
  nanp02aa1n02x5               g078(.a(new_n163), .b(new_n173), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n171), .b(new_n165), .c(new_n172), .o1(new_n175));
  xnrc02aa1n12x5               g080(.a(\b[14] ), .b(\a[15] ), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n174), .c(new_n175), .out0(\s[15] ));
  inv000aa1d42x5               g083(.a(new_n173), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n175), .b(new_n179), .c(new_n160), .d(new_n162), .o1(new_n180));
  norp02aa1n02x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  tech160nm_fixnrc02aa1n02p5x5 g086(.a(\b[15] ), .b(\a[16] ), .out0(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n181), .c(new_n180), .d(new_n177), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n177), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(new_n182), .c(new_n181), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n185), .b(new_n183), .o1(\s[16] ));
  nona22aa1d18x5               g091(.a(new_n173), .b(new_n176), .c(new_n182), .out0(new_n187));
  nor042aa1n12x5               g092(.a(new_n187), .b(new_n158), .o1(new_n188));
  aoai13aa1n12x5               g093(.a(new_n188), .b(new_n123), .c(new_n109), .d(new_n116), .o1(new_n189));
  inv000aa1d42x5               g094(.a(new_n189), .o1(new_n190));
  norp02aa1n02x5               g095(.a(new_n162), .b(new_n187), .o1(new_n191));
  nor002aa1d32x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  nand42aa1n03x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  norp02aa1n02x5               g098(.a(\b[15] ), .b(\a[16] ), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  nona22aa1n02x4               g101(.a(new_n177), .b(new_n182), .c(new_n175), .out0(new_n197));
  nano22aa1n06x5               g102(.a(new_n191), .b(new_n197), .c(new_n196), .out0(new_n198));
  nanp02aa1n06x5               g103(.a(new_n189), .b(new_n198), .o1(new_n199));
  oaib12aa1n02x5               g104(.a(new_n199), .b(new_n192), .c(new_n193), .out0(new_n200));
  norp03aa1n02x5               g105(.a(new_n195), .b(new_n192), .c(new_n194), .o1(new_n201));
  nanp03aa1n02x5               g106(.a(new_n197), .b(new_n193), .c(new_n201), .o1(new_n202));
  oai013aa1n02x4               g107(.a(new_n200), .b(new_n191), .c(new_n190), .d(new_n202), .o1(\s[17] ));
  norp02aa1n06x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  and002aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o(new_n205));
  oai112aa1n02x5               g110(.a(new_n197), .b(new_n201), .c(new_n162), .d(new_n187), .o1(new_n206));
  aoai13aa1n02x5               g111(.a(new_n193), .b(new_n206), .c(new_n138), .d(new_n188), .o1(new_n207));
  nona22aa1n02x4               g112(.a(new_n193), .b(new_n205), .c(new_n204), .out0(new_n208));
  aoib12aa1n02x5               g113(.a(new_n208), .b(new_n189), .c(new_n206), .out0(new_n209));
  oaoi13aa1n02x5               g114(.a(new_n209), .b(new_n207), .c(new_n204), .d(new_n205), .o1(\s[18] ));
  inv000aa1d42x5               g115(.a(new_n192), .o1(new_n211));
  nano23aa1d15x5               g116(.a(new_n205), .b(new_n204), .c(new_n211), .d(new_n193), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoi112aa1n02x5               g118(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n214));
  tech160nm_finor002aa1n03p5x5 g119(.a(new_n214), .b(new_n204), .o1(new_n215));
  aoai13aa1n04x5               g120(.a(new_n215), .b(new_n213), .c(new_n189), .d(new_n198), .o1(new_n216));
  nand42aa1n10x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nor042aa1n04x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  oaoi03aa1n06x5               g124(.a(\a[18] ), .b(\b[17] ), .c(new_n211), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n220), .c(new_n199), .d(new_n212), .o1(new_n221));
  aoi012aa1n02x5               g126(.a(new_n221), .b(new_n216), .c(new_n219), .o1(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand42aa1n08x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanb02aa1n02x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n216), .b(new_n219), .o1(new_n228));
  nona22aa1n02x5               g133(.a(new_n228), .b(new_n226), .c(new_n218), .out0(new_n229));
  nanp02aa1n03x5               g134(.a(new_n229), .b(new_n227), .o1(\s[20] ));
  oai112aa1n03x5               g135(.a(new_n196), .b(new_n197), .c(new_n162), .d(new_n187), .o1(new_n231));
  nona23aa1n09x5               g136(.a(new_n217), .b(new_n225), .c(new_n224), .d(new_n218), .out0(new_n232));
  norb02aa1n03x5               g137(.a(new_n212), .b(new_n232), .out0(new_n233));
  aoai13aa1n02x5               g138(.a(new_n233), .b(new_n231), .c(new_n138), .d(new_n188), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n233), .o1(new_n235));
  oa0012aa1n03x5               g140(.a(new_n225), .b(new_n224), .c(new_n218), .o(new_n236));
  oabi12aa1n09x5               g141(.a(new_n236), .b(new_n215), .c(new_n232), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoai13aa1n04x5               g143(.a(new_n238), .b(new_n235), .c(new_n189), .d(new_n198), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nano23aa1n03x7               g146(.a(new_n224), .b(new_n218), .c(new_n225), .d(new_n217), .out0(new_n242));
  aoi112aa1n02x5               g147(.a(new_n236), .b(new_n241), .c(new_n242), .d(new_n220), .o1(new_n243));
  aoi022aa1n02x5               g148(.a(new_n239), .b(new_n241), .c(new_n234), .d(new_n243), .o1(\s[21] ));
  norp02aa1n02x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  xnrc02aa1n12x5               g150(.a(\b[21] ), .b(\a[22] ), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n245), .c(new_n239), .d(new_n241), .o1(new_n247));
  nand22aa1n02x5               g152(.a(new_n239), .b(new_n241), .o1(new_n248));
  nona22aa1n02x5               g153(.a(new_n248), .b(new_n246), .c(new_n245), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n249), .b(new_n247), .o1(\s[22] ));
  inv000aa1d42x5               g155(.a(\a[22] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\b[21] ), .o1(new_n252));
  oao003aa1n06x5               g157(.a(new_n251), .b(new_n252), .c(new_n245), .carry(new_n253));
  norp02aa1n02x5               g158(.a(\b[22] ), .b(\a[23] ), .o1(new_n254));
  nanp02aa1n02x5               g159(.a(\b[22] ), .b(\a[23] ), .o1(new_n255));
  nanb02aa1n02x5               g160(.a(new_n254), .b(new_n255), .out0(new_n256));
  nona32aa1n03x5               g161(.a(new_n199), .b(new_n246), .c(new_n240), .d(new_n235), .out0(new_n257));
  nor042aa1n03x5               g162(.a(new_n246), .b(new_n240), .o1(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n236), .c(new_n242), .d(new_n220), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n253), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n259), .b(new_n260), .o1(new_n261));
  oaib12aa1n03x5               g166(.a(new_n256), .b(new_n261), .c(new_n257), .out0(new_n262));
  nanp03aa1n02x5               g167(.a(new_n212), .b(new_n258), .c(new_n242), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n259), .b(new_n263), .c(new_n189), .d(new_n198), .o1(new_n264));
  oai013aa1n03x4               g169(.a(new_n262), .b(new_n256), .c(new_n253), .d(new_n264), .o1(\s[23] ));
  norp02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  nanb02aa1n02x5               g172(.a(new_n266), .b(new_n267), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n259), .o1(new_n269));
  nona32aa1n02x5               g174(.a(new_n257), .b(new_n254), .c(new_n253), .d(new_n269), .out0(new_n270));
  oai012aa1n02x5               g175(.a(new_n260), .b(\b[22] ), .c(\a[23] ), .o1(new_n271));
  oai012aa1n03x5               g176(.a(new_n255), .b(new_n264), .c(new_n271), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n266), .b(new_n255), .c(new_n267), .out0(new_n273));
  aoi022aa1n03x5               g178(.a(new_n272), .b(new_n268), .c(new_n270), .d(new_n273), .o1(\s[24] ));
  nona32aa1n02x4               g179(.a(new_n199), .b(new_n268), .c(new_n256), .d(new_n263), .out0(new_n275));
  nano23aa1n02x4               g180(.a(new_n254), .b(new_n266), .c(new_n267), .d(new_n255), .out0(new_n276));
  inv000aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  nona23aa1n02x4               g182(.a(new_n258), .b(new_n212), .c(new_n277), .d(new_n232), .out0(new_n278));
  oai012aa1n02x5               g183(.a(new_n267), .b(new_n266), .c(new_n254), .o1(new_n279));
  aoai13aa1n09x5               g184(.a(new_n279), .b(new_n277), .c(new_n259), .d(new_n260), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n281), .b(new_n278), .c(new_n189), .d(new_n198), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  aoai13aa1n02x7               g188(.a(new_n276), .b(new_n253), .c(new_n237), .d(new_n258), .o1(new_n284));
  nano22aa1n02x4               g189(.a(new_n283), .b(new_n284), .c(new_n279), .out0(new_n285));
  aoi022aa1n02x5               g190(.a(new_n282), .b(new_n283), .c(new_n275), .d(new_n285), .o1(\s[25] ));
  norp02aa1n02x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[25] ), .b(\a[26] ), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n287), .c(new_n282), .d(new_n283), .o1(new_n289));
  nanp02aa1n02x5               g194(.a(new_n282), .b(new_n283), .o1(new_n290));
  nona22aa1n02x5               g195(.a(new_n290), .b(new_n288), .c(new_n287), .out0(new_n291));
  nand42aa1n02x5               g196(.a(new_n291), .b(new_n289), .o1(\s[26] ));
  norb02aa1n03x5               g197(.a(new_n283), .b(new_n288), .out0(new_n293));
  nano22aa1n03x7               g198(.a(new_n263), .b(new_n276), .c(new_n293), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n231), .c(new_n138), .d(new_n188), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(\b[25] ), .b(\a[26] ), .o1(new_n296));
  oai022aa1n02x5               g201(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n297));
  aoi022aa1n06x5               g202(.a(new_n280), .b(new_n293), .c(new_n296), .d(new_n297), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  nand22aa1n03x5               g204(.a(new_n280), .b(new_n293), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n297), .b(new_n296), .o1(new_n301));
  and002aa1n02x5               g206(.a(new_n299), .b(new_n301), .o(new_n302));
  nanp03aa1n02x5               g207(.a(new_n295), .b(new_n300), .c(new_n302), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n299), .c(new_n295), .d(new_n298), .o1(\s[27] ));
  xnrc02aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .out0(new_n305));
  inv000aa1d42x5               g210(.a(\a[27] ), .o1(new_n306));
  inv000aa1d42x5               g211(.a(\b[26] ), .o1(new_n307));
  aoi022aa1n02x5               g212(.a(new_n297), .b(new_n296), .c(new_n306), .d(new_n307), .o1(new_n308));
  nanp03aa1n06x5               g213(.a(new_n295), .b(new_n300), .c(new_n308), .o1(new_n309));
  oaib12aa1n06x5               g214(.a(new_n309), .b(new_n307), .c(\a[27] ), .out0(new_n310));
  oai022aa1n02x5               g215(.a(new_n306), .b(new_n307), .c(\b[27] ), .d(\a[28] ), .o1(new_n311));
  aoi012aa1n02x5               g216(.a(new_n311), .b(\a[28] ), .c(\b[27] ), .o1(new_n312));
  aoi022aa1n02x5               g217(.a(new_n310), .b(new_n305), .c(new_n309), .d(new_n312), .o1(\s[28] ));
  aobi12aa1n06x5               g218(.a(new_n294), .b(new_n189), .c(new_n198), .out0(new_n314));
  inv000aa1d42x5               g219(.a(new_n293), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n301), .b(new_n315), .c(new_n284), .d(new_n279), .o1(new_n316));
  inv000aa1d42x5               g221(.a(\a[28] ), .o1(new_n317));
  xroi22aa1d04x5               g222(.a(new_n306), .b(\b[26] ), .c(new_n317), .d(\b[27] ), .out0(new_n318));
  oaih12aa1n02x5               g223(.a(new_n318), .b(new_n316), .c(new_n314), .o1(new_n319));
  inv040aa1n03x5               g224(.a(new_n318), .o1(new_n320));
  nanp02aa1n02x5               g225(.a(new_n307), .b(new_n306), .o1(new_n321));
  oao003aa1n03x5               g226(.a(\a[28] ), .b(\b[27] ), .c(new_n321), .carry(new_n322));
  aoai13aa1n04x5               g227(.a(new_n322), .b(new_n320), .c(new_n298), .d(new_n295), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .out0(new_n324));
  norb02aa1n02x5               g229(.a(new_n322), .b(new_n324), .out0(new_n325));
  aoi022aa1n03x5               g230(.a(new_n323), .b(new_n324), .c(new_n319), .d(new_n325), .o1(\s[29] ));
  xnrb03aa1n02x5               g231(.a(new_n131), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g232(.a(new_n305), .b(new_n299), .c(new_n324), .out0(new_n328));
  oaih12aa1n02x5               g233(.a(new_n328), .b(new_n316), .c(new_n314), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n328), .o1(new_n330));
  oaoi03aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n322), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n331), .o1(new_n332));
  aoai13aa1n02x7               g237(.a(new_n332), .b(new_n330), .c(new_n298), .d(new_n295), .o1(new_n333));
  xorc02aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .out0(new_n334));
  aoi012aa1n02x5               g239(.a(new_n322), .b(\a[29] ), .c(\b[28] ), .o1(new_n335));
  oabi12aa1n02x5               g240(.a(new_n334), .b(\a[29] ), .c(\b[28] ), .out0(new_n336));
  norp02aa1n02x5               g241(.a(new_n335), .b(new_n336), .o1(new_n337));
  aoi022aa1n02x7               g242(.a(new_n333), .b(new_n334), .c(new_n329), .d(new_n337), .o1(\s[30] ));
  nano22aa1n02x4               g243(.a(new_n320), .b(new_n324), .c(new_n334), .out0(new_n339));
  oaih12aa1n02x5               g244(.a(new_n339), .b(new_n316), .c(new_n314), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[31] ), .b(\b[30] ), .out0(new_n341));
  inv000aa1d42x5               g246(.a(\a[30] ), .o1(new_n342));
  inv000aa1d42x5               g247(.a(\b[29] ), .o1(new_n343));
  oabi12aa1n02x5               g248(.a(new_n341), .b(\a[30] ), .c(\b[29] ), .out0(new_n344));
  oaoi13aa1n02x5               g249(.a(new_n344), .b(new_n331), .c(new_n342), .d(new_n343), .o1(new_n345));
  inv000aa1n02x5               g250(.a(new_n339), .o1(new_n346));
  oaoi03aa1n02x5               g251(.a(new_n342), .b(new_n343), .c(new_n331), .o1(new_n347));
  aoai13aa1n02x7               g252(.a(new_n347), .b(new_n346), .c(new_n298), .d(new_n295), .o1(new_n348));
  aoi022aa1n03x5               g253(.a(new_n348), .b(new_n341), .c(new_n340), .d(new_n345), .o1(\s[31] ));
  norb02aa1n02x5               g254(.a(new_n106), .b(new_n101), .out0(new_n350));
  xobna2aa1n03x5               g255(.a(new_n350), .b(new_n132), .c(new_n104), .out0(\s[3] ));
  aoai13aa1n02x5               g256(.a(new_n350), .b(new_n105), .c(\a[2] ), .d(\b[1] ), .o1(new_n352));
  xnrc02aa1n02x5               g257(.a(\b[3] ), .b(\a[4] ), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n352), .c(new_n106), .out0(\s[4] ));
  xorc02aa1n02x5               g259(.a(\a[5] ), .b(\b[4] ), .out0(new_n355));
  nanp03aa1n02x5               g260(.a(new_n133), .b(new_n102), .c(new_n355), .o1(new_n356));
  oaib12aa1n02x5               g261(.a(new_n356), .b(new_n355), .c(new_n109), .out0(\s[5] ));
  nanp02aa1n02x5               g262(.a(\b[4] ), .b(\a[5] ), .o1(new_n358));
  xorc02aa1n02x5               g263(.a(\a[6] ), .b(\b[5] ), .out0(new_n359));
  inv000aa1d42x5               g264(.a(new_n359), .o1(new_n360));
  xnbna2aa1n03x5               g265(.a(new_n360), .b(new_n356), .c(new_n358), .out0(\s[6] ));
  aob012aa1n02x5               g266(.a(new_n359), .b(new_n356), .c(new_n358), .out0(new_n362));
  norb02aa1n02x5               g267(.a(new_n117), .b(new_n118), .out0(new_n363));
  xobna2aa1n03x5               g268(.a(new_n363), .b(new_n362), .c(new_n114), .out0(\s[7] ));
  aoai13aa1n02x5               g269(.a(new_n114), .b(new_n360), .c(new_n356), .d(new_n358), .o1(new_n365));
  oaoi03aa1n02x5               g270(.a(\a[7] ), .b(\b[6] ), .c(new_n365), .o1(new_n366));
  xorb03aa1n02x5               g271(.a(new_n366), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  nanp02aa1n02x5               g272(.a(new_n109), .b(new_n116), .o1(new_n368));
  norp03aa1n02x5               g273(.a(new_n136), .b(new_n121), .c(new_n124), .o1(new_n369));
  aoi022aa1n02x5               g274(.a(new_n138), .b(new_n124), .c(new_n368), .d(new_n369), .o1(\s[9] ));
endmodule


