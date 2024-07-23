// Benchmark "adder" written by ABC on Wed Jul 17 20:59:53 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n172, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n331, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n350, new_n351,
    new_n353, new_n355, new_n356, new_n357, new_n359, new_n360, new_n362,
    new_n363, new_n365, new_n367, new_n368;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand42aa1n08x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n06x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  norb03aa1n12x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  nand22aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  inv040aa1d32x5               g008(.a(\a[3] ), .o1(new_n104));
  inv040aa1d28x5               g009(.a(\b[2] ), .o1(new_n105));
  nanp02aa1n04x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  nand23aa1n04x5               g011(.a(new_n106), .b(new_n100), .c(new_n103), .o1(new_n107));
  nor042aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  and002aa1n12x5               g013(.a(\b[3] ), .b(\a[4] ), .o(new_n109));
  aoi112aa1n06x5               g014(.a(new_n109), .b(new_n108), .c(new_n104), .d(new_n105), .o1(new_n110));
  oai012aa1d24x5               g015(.a(new_n110), .b(new_n102), .c(new_n107), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nor042aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand42aa1d28x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norb02aa1n03x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  aoi022aa1d24x5               g020(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n116));
  inv040aa1d32x5               g021(.a(\a[5] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\b[4] ), .o1(new_n118));
  aoi022aa1d24x5               g023(.a(new_n118), .b(new_n117), .c(\a[4] ), .d(\b[3] ), .o1(new_n119));
  nor002aa1d32x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  inv040aa1n03x5               g025(.a(new_n120), .o1(new_n121));
  orn002aa1n24x5               g026(.a(\a[6] ), .b(\b[5] ), .o(new_n122));
  nanp03aa1n02x5               g027(.a(new_n119), .b(new_n121), .c(new_n122), .o1(new_n123));
  nano32aa1n09x5               g028(.a(new_n123), .b(new_n115), .c(new_n116), .d(new_n112), .out0(new_n124));
  tech160nm_fioai012aa1n03p5x5 g029(.a(new_n114), .b(new_n120), .c(new_n113), .o1(new_n125));
  tech160nm_finand02aa1n03p5x5 g030(.a(\b[5] ), .b(\a[6] ), .o1(new_n126));
  oai112aa1n06x5               g031(.a(new_n122), .b(new_n126), .c(\b[4] ), .d(\a[5] ), .o1(new_n127));
  nona23aa1d16x5               g032(.a(new_n116), .b(new_n114), .c(new_n120), .d(new_n113), .out0(new_n128));
  oaib12aa1n18x5               g033(.a(new_n125), .b(new_n128), .c(new_n127), .out0(new_n129));
  nand42aa1n08x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  norb02aa1n06x5               g035(.a(new_n130), .b(new_n97), .out0(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n129), .c(new_n111), .d(new_n124), .o1(new_n132));
  norp02aa1n04x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  norb02aa1n03x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  oai022aa1d24x5               g040(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n136));
  nanb03aa1n02x5               g041(.a(new_n136), .b(new_n132), .c(new_n134), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n135), .c(new_n98), .d(new_n132), .o1(\s[10] ));
  nano23aa1n03x5               g043(.a(new_n97), .b(new_n133), .c(new_n134), .d(new_n130), .out0(new_n139));
  aoai13aa1n06x5               g044(.a(new_n139), .b(new_n129), .c(new_n111), .d(new_n124), .o1(new_n140));
  oai012aa1n02x5               g045(.a(new_n134), .b(new_n133), .c(new_n97), .o1(new_n141));
  norp02aa1n04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nand42aa1d28x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n12x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n140), .c(new_n141), .out0(\s[11] ));
  nor042aa1d18x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  aobi12aa1n02x5               g051(.a(new_n144), .b(new_n140), .c(new_n141), .out0(new_n147));
  nand02aa1d16x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n09x5               g053(.a(new_n148), .b(new_n146), .out0(new_n149));
  inv000aa1n02x5               g054(.a(new_n149), .o1(new_n150));
  oai012aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n142), .o1(new_n151));
  nanb02aa1n03x5               g056(.a(new_n147), .b(new_n148), .out0(new_n152));
  oai013aa1n02x4               g057(.a(new_n151), .b(new_n152), .c(new_n142), .d(new_n146), .o1(\s[12] ));
  nanp02aa1n03x5               g058(.a(new_n124), .b(new_n111), .o1(new_n154));
  nanb02aa1n06x5               g059(.a(new_n129), .b(new_n154), .out0(new_n155));
  nano32aa1n03x7               g060(.a(new_n150), .b(new_n144), .c(new_n135), .d(new_n131), .out0(new_n156));
  inv040aa1d32x5               g061(.a(\a[11] ), .o1(new_n157));
  inv040aa1d28x5               g062(.a(\b[10] ), .o1(new_n158));
  aoai13aa1n12x5               g063(.a(new_n148), .b(new_n146), .c(new_n157), .d(new_n158), .o1(new_n159));
  nanb03aa1d24x5               g064(.a(new_n146), .b(new_n148), .c(new_n143), .out0(new_n160));
  nanp02aa1n04x5               g065(.a(new_n158), .b(new_n157), .o1(new_n161));
  nanp03aa1d12x5               g066(.a(new_n136), .b(new_n161), .c(new_n134), .o1(new_n162));
  oai012aa1n12x5               g067(.a(new_n159), .b(new_n162), .c(new_n160), .o1(new_n163));
  nor002aa1d32x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand02aa1n12x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n06x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n163), .c(new_n155), .d(new_n156), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n166), .o1(new_n168));
  oai112aa1n02x5               g073(.a(new_n159), .b(new_n168), .c(new_n162), .d(new_n160), .o1(new_n169));
  aoi012aa1n02x5               g074(.a(new_n169), .b(new_n155), .c(new_n156), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n167), .b(new_n170), .out0(\s[13] ));
  inv000aa1d42x5               g076(.a(new_n164), .o1(new_n172));
  nor002aa1n16x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand02aa1n08x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nona23aa1n03x5               g080(.a(new_n167), .b(new_n174), .c(new_n173), .d(new_n164), .out0(new_n176));
  aoai13aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n172), .d(new_n167), .o1(\s[14] ));
  aoi012aa1d18x5               g082(.a(new_n129), .b(new_n124), .c(new_n111), .o1(new_n178));
  nano23aa1d15x5               g083(.a(new_n164), .b(new_n173), .c(new_n174), .d(new_n165), .out0(new_n179));
  nano22aa1n03x7               g084(.a(new_n178), .b(new_n156), .c(new_n179), .out0(new_n180));
  nona23aa1n09x5               g085(.a(new_n174), .b(new_n165), .c(new_n164), .d(new_n173), .out0(new_n181));
  oaoi13aa1n04x5               g086(.a(new_n181), .b(new_n159), .c(new_n162), .d(new_n160), .o1(new_n182));
  oaoi03aa1n06x5               g087(.a(\a[14] ), .b(\b[13] ), .c(new_n172), .o1(new_n183));
  tech160nm_fixorc02aa1n05x5   g088(.a(\a[15] ), .b(\b[14] ), .out0(new_n184));
  oai013aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n182), .d(new_n183), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n183), .c(new_n163), .d(new_n179), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n180), .out0(new_n187));
  norb02aa1n02x5               g092(.a(new_n185), .b(new_n187), .out0(\s[15] ));
  nor042aa1n03x5               g093(.a(\b[14] ), .b(\a[15] ), .o1(new_n189));
  inv040aa1n02x5               g094(.a(new_n189), .o1(new_n190));
  tech160nm_fixorc02aa1n02p5x5 g095(.a(\a[16] ), .b(\b[15] ), .out0(new_n191));
  and002aa1n02x5               g096(.a(\b[15] ), .b(\a[16] ), .o(new_n192));
  oai022aa1n02x5               g097(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n193));
  nona22aa1n02x4               g098(.a(new_n185), .b(new_n192), .c(new_n193), .out0(new_n194));
  aoai13aa1n02x5               g099(.a(new_n194), .b(new_n191), .c(new_n185), .d(new_n190), .o1(\s[16] ));
  nand03aa1n02x5               g100(.a(new_n139), .b(new_n144), .c(new_n149), .o1(new_n196));
  nano32aa1n03x7               g101(.a(new_n196), .b(new_n191), .c(new_n179), .d(new_n184), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n129), .c(new_n111), .d(new_n124), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n191), .b(new_n184), .o1(new_n199));
  nona22aa1n06x5               g104(.a(new_n156), .b(new_n199), .c(new_n181), .out0(new_n200));
  and002aa1n06x5               g105(.a(new_n191), .b(new_n184), .o(new_n201));
  aoai13aa1n09x5               g106(.a(new_n201), .b(new_n183), .c(new_n163), .d(new_n179), .o1(new_n202));
  oaoi03aa1n12x5               g107(.a(\a[16] ), .b(\b[15] ), .c(new_n190), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  oai112aa1n06x5               g109(.a(new_n202), .b(new_n204), .c(new_n178), .d(new_n200), .o1(new_n205));
  xorc02aa1n12x5               g110(.a(\a[17] ), .b(\b[16] ), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  and003aa1n02x5               g112(.a(new_n202), .b(new_n207), .c(new_n204), .o(new_n208));
  aoi022aa1n02x5               g113(.a(new_n208), .b(new_n198), .c(new_n205), .d(new_n206), .o1(\s[17] ));
  nor042aa1n06x5               g114(.a(new_n178), .b(new_n200), .o1(new_n210));
  nanp02aa1n09x5               g115(.a(new_n202), .b(new_n204), .o1(new_n211));
  norp02aa1n02x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  oaoi13aa1n02x5               g117(.a(new_n212), .b(new_n206), .c(new_n211), .d(new_n210), .o1(new_n213));
  tech160nm_fixorc02aa1n05x5   g118(.a(\a[18] ), .b(\b[17] ), .out0(new_n214));
  oaoi13aa1n04x5               g119(.a(new_n203), .b(new_n201), .c(new_n182), .d(new_n183), .o1(new_n215));
  oai022aa1n09x5               g120(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n216));
  aoi012aa1n02x5               g121(.a(new_n216), .b(\a[18] ), .c(\b[17] ), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n207), .c(new_n215), .d(new_n198), .o1(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n213), .c(new_n214), .o1(\s[18] ));
  and002aa1n06x5               g124(.a(new_n214), .b(new_n206), .o(new_n220));
  oai012aa1n06x5               g125(.a(new_n220), .b(new_n211), .c(new_n210), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\a[18] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[17] ), .o1(new_n223));
  oao003aa1n02x5               g128(.a(new_n222), .b(new_n223), .c(new_n212), .carry(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xorc02aa1n12x5               g130(.a(\a[19] ), .b(\b[18] ), .out0(new_n226));
  xnbna2aa1n03x5               g131(.a(new_n226), .b(new_n221), .c(new_n225), .out0(\s[19] ));
  xnrc02aa1n02x5               g132(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g133(.a(\a[19] ), .o1(new_n229));
  nanb02aa1n02x5               g134(.a(\b[18] ), .b(new_n229), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n226), .b(new_n224), .c(new_n205), .d(new_n220), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[20] ), .b(\b[19] ), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n226), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(\b[19] ), .b(\a[20] ), .o1(new_n234));
  oai022aa1d18x5               g139(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n234), .b(new_n235), .out0(new_n236));
  aoai13aa1n02x7               g141(.a(new_n236), .b(new_n233), .c(new_n221), .d(new_n225), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n232), .c(new_n231), .d(new_n230), .o1(\s[20] ));
  inv020aa1n04x5               g143(.a(\a[20] ), .o1(new_n239));
  xroi22aa1d06x4               g144(.a(new_n229), .b(\b[18] ), .c(new_n239), .d(\b[19] ), .out0(new_n240));
  nand23aa1d12x5               g145(.a(new_n240), .b(new_n206), .c(new_n214), .o1(new_n241));
  aoi022aa1n09x5               g146(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n242));
  aoi012aa1n09x5               g147(.a(new_n235), .b(new_n216), .c(new_n242), .o1(new_n243));
  norb02aa1n06x4               g148(.a(new_n234), .b(new_n243), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoai13aa1n02x7               g150(.a(new_n245), .b(new_n241), .c(new_n215), .d(new_n198), .o1(new_n246));
  nor022aa1n08x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  nand42aa1d28x5               g152(.a(\b[20] ), .b(\a[21] ), .o1(new_n248));
  norb02aa1n06x5               g153(.a(new_n248), .b(new_n247), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n241), .o1(new_n250));
  aoi112aa1n03x4               g155(.a(new_n249), .b(new_n244), .c(new_n205), .d(new_n250), .o1(new_n251));
  tech160nm_fiaoi012aa1n02p5x5 g156(.a(new_n251), .b(new_n246), .c(new_n249), .o1(\s[21] ));
  xnrc02aa1n02x5               g157(.a(\b[21] ), .b(\a[22] ), .out0(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n247), .c(new_n246), .d(new_n248), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n249), .b(new_n244), .c(new_n205), .d(new_n250), .o1(new_n255));
  nand02aa1n10x5               g160(.a(\b[21] ), .b(\a[22] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  oai022aa1n02x5               g162(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n258));
  nona22aa1n03x5               g163(.a(new_n255), .b(new_n257), .c(new_n258), .out0(new_n259));
  nanp02aa1n03x5               g164(.a(new_n254), .b(new_n259), .o1(\s[22] ));
  norb02aa1n03x4               g165(.a(new_n249), .b(new_n253), .out0(new_n261));
  and003aa1n02x5               g166(.a(new_n220), .b(new_n261), .c(new_n240), .o(new_n262));
  tech160nm_fioai012aa1n05x5   g167(.a(new_n262), .b(new_n211), .c(new_n210), .o1(new_n263));
  aoi012aa1n06x5               g168(.a(new_n247), .b(\a[20] ), .c(\b[19] ), .o1(new_n264));
  oai012aa1n03x5               g169(.a(new_n248), .b(\b[21] ), .c(\a[22] ), .o1(new_n265));
  nanb03aa1n09x5               g170(.a(new_n265), .b(new_n264), .c(new_n256), .out0(new_n266));
  nand42aa1n02x5               g171(.a(new_n258), .b(new_n256), .o1(new_n267));
  oai012aa1n12x5               g172(.a(new_n267), .b(new_n266), .c(new_n243), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(new_n263), .b(new_n269), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[23] ), .b(\b[22] ), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  oai112aa1n02x5               g177(.a(new_n267), .b(new_n272), .c(new_n266), .d(new_n243), .o1(new_n273));
  aboi22aa1n03x5               g178(.a(new_n273), .b(new_n263), .c(new_n270), .d(new_n271), .out0(\s[23] ));
  nor042aa1n06x5               g179(.a(\b[22] ), .b(\a[23] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n271), .b(new_n268), .c(new_n205), .d(new_n262), .o1(new_n277));
  tech160nm_fixorc02aa1n03p5x5 g182(.a(\a[24] ), .b(\b[23] ), .out0(new_n278));
  oai022aa1n02x5               g183(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(\a[24] ), .c(\b[23] ), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n280), .b(new_n272), .c(new_n263), .d(new_n269), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n277), .d(new_n276), .o1(\s[24] ));
  nand02aa1d08x5               g187(.a(new_n278), .b(new_n271), .o1(new_n283));
  inv000aa1n06x5               g188(.a(new_n283), .o1(new_n284));
  nano22aa1n12x5               g189(.a(new_n241), .b(new_n284), .c(new_n261), .out0(new_n285));
  tech160nm_fioai012aa1n04x5   g190(.a(new_n285), .b(new_n211), .c(new_n210), .o1(new_n286));
  tech160nm_fioaoi03aa1n03p5x5 g191(.a(\a[24] ), .b(\b[23] ), .c(new_n276), .o1(new_n287));
  tech160nm_fiaoi012aa1n05x5   g192(.a(new_n287), .b(new_n268), .c(new_n284), .o1(new_n288));
  inv040aa1n03x5               g193(.a(new_n288), .o1(new_n289));
  xorc02aa1n12x5               g194(.a(\a[25] ), .b(\b[24] ), .out0(new_n290));
  aoai13aa1n06x5               g195(.a(new_n290), .b(new_n289), .c(new_n205), .d(new_n285), .o1(new_n291));
  aoi112aa1n02x5               g196(.a(new_n290), .b(new_n287), .c(new_n268), .d(new_n284), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n291), .b(new_n292), .c(new_n286), .out0(\s[25] ));
  nor042aa1n03x5               g198(.a(\b[24] ), .b(\a[25] ), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(new_n290), .o1(new_n297));
  oai022aa1n02x5               g202(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(\a[26] ), .c(\b[25] ), .o1(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n297), .c(new_n286), .d(new_n288), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n296), .c(new_n291), .d(new_n295), .o1(\s[26] ));
  and002aa1n06x5               g206(.a(new_n296), .b(new_n290), .o(new_n302));
  nano32aa1n03x7               g207(.a(new_n241), .b(new_n302), .c(new_n261), .d(new_n284), .out0(new_n303));
  oai012aa1n06x5               g208(.a(new_n303), .b(new_n211), .c(new_n210), .o1(new_n304));
  inv020aa1n03x5               g209(.a(new_n303), .o1(new_n305));
  oaoi13aa1n06x5               g210(.a(new_n283), .b(new_n267), .c(new_n266), .d(new_n243), .o1(new_n306));
  oaoi03aa1n02x5               g211(.a(\a[26] ), .b(\b[25] ), .c(new_n295), .o1(new_n307));
  oaoi13aa1n09x5               g212(.a(new_n307), .b(new_n302), .c(new_n306), .d(new_n287), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n308), .b(new_n305), .c(new_n215), .d(new_n198), .o1(new_n309));
  xorc02aa1n12x5               g214(.a(\a[27] ), .b(\b[26] ), .out0(new_n310));
  aoi112aa1n03x4               g215(.a(new_n310), .b(new_n307), .c(new_n289), .d(new_n302), .o1(new_n311));
  aoi022aa1n03x5               g216(.a(new_n309), .b(new_n310), .c(new_n311), .d(new_n304), .o1(\s[27] ));
  norp02aa1n02x5               g217(.a(\b[26] ), .b(\a[27] ), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  nand02aa1n02x5               g219(.a(new_n309), .b(new_n310), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .out0(new_n316));
  inv000aa1d42x5               g221(.a(new_n310), .o1(new_n317));
  oai022aa1d24x5               g222(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n318));
  aoi012aa1n02x5               g223(.a(new_n318), .b(\a[28] ), .c(\b[27] ), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n319), .b(new_n317), .c(new_n304), .d(new_n308), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n315), .d(new_n314), .o1(\s[28] ));
  and002aa1n02x5               g226(.a(new_n316), .b(new_n310), .o(new_n322));
  inv000aa1d42x5               g227(.a(new_n322), .o1(new_n323));
  inv000aa1d42x5               g228(.a(\b[27] ), .o1(new_n324));
  oaib12aa1n18x5               g229(.a(new_n318), .b(new_n324), .c(\a[28] ), .out0(new_n325));
  inv000aa1d42x5               g230(.a(new_n325), .o1(new_n326));
  tech160nm_fixorc02aa1n03p5x5 g231(.a(\a[29] ), .b(\b[28] ), .out0(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(new_n328));
  aoai13aa1n02x7               g233(.a(new_n328), .b(new_n323), .c(new_n304), .d(new_n308), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n327), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n309), .d(new_n322), .o1(new_n331));
  nanp02aa1n03x5               g236(.a(new_n331), .b(new_n329), .o1(\s[29] ));
  xorb03aa1n02x5               g237(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g238(.a(new_n330), .b(new_n310), .c(new_n316), .out0(new_n334));
  tech160nm_fioaoi03aa1n03p5x5 g239(.a(\a[29] ), .b(\b[28] ), .c(new_n325), .o1(new_n335));
  xnrc02aa1n02x5               g240(.a(\b[29] ), .b(\a[30] ), .out0(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n309), .d(new_n334), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n334), .o1(new_n338));
  norp02aa1n03x5               g243(.a(new_n335), .b(new_n336), .o1(new_n339));
  aoai13aa1n02x7               g244(.a(new_n339), .b(new_n338), .c(new_n304), .d(new_n308), .o1(new_n340));
  nanp02aa1n03x5               g245(.a(new_n337), .b(new_n340), .o1(\s[30] ));
  nano32aa1n03x7               g246(.a(new_n336), .b(new_n327), .c(new_n316), .d(new_n310), .out0(new_n342));
  aoi012aa1n02x5               g247(.a(new_n339), .b(\a[30] ), .c(\b[29] ), .o1(new_n343));
  xnrc02aa1n02x5               g248(.a(\b[30] ), .b(\a[31] ), .out0(new_n344));
  aoai13aa1n03x5               g249(.a(new_n344), .b(new_n343), .c(new_n309), .d(new_n342), .o1(new_n345));
  inv000aa1d42x5               g250(.a(new_n342), .o1(new_n346));
  norp02aa1n02x5               g251(.a(new_n343), .b(new_n344), .o1(new_n347));
  aoai13aa1n02x7               g252(.a(new_n347), .b(new_n346), .c(new_n304), .d(new_n308), .o1(new_n348));
  nanp02aa1n03x5               g253(.a(new_n345), .b(new_n348), .o1(\s[31] ));
  norp02aa1n02x5               g254(.a(new_n102), .b(new_n107), .o1(new_n350));
  aboi22aa1n03x5               g255(.a(new_n102), .b(new_n100), .c(new_n106), .d(new_n103), .out0(new_n351));
  norp02aa1n02x5               g256(.a(new_n351), .b(new_n350), .o1(\s[3] ));
  obai22aa1n02x7               g257(.a(new_n106), .b(new_n350), .c(new_n108), .d(new_n109), .out0(new_n353));
  nanp02aa1n02x5               g258(.a(new_n353), .b(new_n111), .o1(\s[4] ));
  nanp02aa1n02x5               g259(.a(new_n118), .b(new_n117), .o1(new_n355));
  aboi22aa1n03x5               g260(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n355), .out0(new_n356));
  nanp03aa1n03x5               g261(.a(new_n111), .b(new_n112), .c(new_n119), .o1(new_n357));
  norb02aa1n02x5               g262(.a(new_n357), .b(new_n356), .out0(\s[5] ));
  xorc02aa1n02x5               g263(.a(\a[6] ), .b(\b[5] ), .out0(new_n359));
  nanb02aa1n06x5               g264(.a(new_n127), .b(new_n357), .out0(new_n360));
  aoai13aa1n02x5               g265(.a(new_n360), .b(new_n359), .c(new_n355), .d(new_n357), .o1(\s[6] ));
  nanp02aa1n02x5               g266(.a(\b[6] ), .b(\a[7] ), .o1(new_n362));
  aoi022aa1n02x5               g267(.a(new_n360), .b(new_n126), .c(new_n362), .d(new_n121), .o1(new_n363));
  aoi013aa1n02x4               g268(.a(new_n363), .b(new_n360), .c(new_n121), .d(new_n116), .o1(\s[7] ));
  nanp03aa1n03x5               g269(.a(new_n360), .b(new_n116), .c(new_n121), .o1(new_n365));
  xnbna2aa1n03x5               g270(.a(new_n115), .b(new_n365), .c(new_n121), .out0(\s[8] ));
  oaib12aa1n02x5               g271(.a(new_n125), .b(new_n97), .c(new_n130), .out0(new_n367));
  aoib12aa1n02x5               g272(.a(new_n367), .b(new_n127), .c(new_n128), .out0(new_n368));
  aoi022aa1n02x5               g273(.a(new_n155), .b(new_n131), .c(new_n154), .d(new_n368), .o1(\s[9] ));
endmodule


