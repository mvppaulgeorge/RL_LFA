// Benchmark "adder" written by ABC on Wed Jul 17 22:19:09 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n345, new_n346, new_n347, new_n349, new_n351, new_n354, new_n355,
    new_n356, new_n358, new_n359, new_n361, new_n362;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1n12x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x4               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[4] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[3] ), .o1(new_n103));
  norp02aa1n09x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  tech160nm_fioaoi03aa1n03p5x5 g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor042aa1n06x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand42aa1n10x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand22aa1n09x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  oai122aa1n06x5               g013(.a(new_n107), .b(new_n106), .c(new_n108), .d(\b[3] ), .e(\a[4] ), .o1(new_n109));
  nand42aa1n08x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nand02aa1d24x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanb03aa1n02x5               g016(.a(new_n104), .b(new_n111), .c(new_n110), .out0(new_n112));
  oai012aa1n06x5               g017(.a(new_n105), .b(new_n109), .c(new_n112), .o1(new_n113));
  nor042aa1d18x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand42aa1n20x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanb02aa1n06x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nor042aa1d18x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand02aa1d04x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nanb02aa1n06x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nor042aa1n04x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand02aa1d08x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nor002aa1n12x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nand22aa1n09x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  nona23aa1n09x5               g028(.a(new_n123), .b(new_n121), .c(new_n120), .d(new_n122), .out0(new_n124));
  nor043aa1n04x5               g029(.a(new_n124), .b(new_n119), .c(new_n116), .o1(new_n125));
  nano22aa1n03x7               g030(.a(new_n122), .b(new_n121), .c(new_n123), .out0(new_n126));
  oai012aa1n06x5               g031(.a(new_n115), .b(\b[7] ), .c(\a[8] ), .o1(new_n127));
  oab012aa1n03x5               g032(.a(new_n127), .b(new_n114), .c(new_n117), .out0(new_n128));
  aoi012aa1n06x5               g033(.a(new_n120), .b(new_n122), .c(new_n121), .o1(new_n129));
  aob012aa1n06x5               g034(.a(new_n129), .b(new_n128), .c(new_n126), .out0(new_n130));
  xorc02aa1n12x5               g035(.a(\a[9] ), .b(\b[8] ), .out0(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n130), .c(new_n113), .d(new_n125), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n99), .b(new_n132), .c(new_n101), .out0(\s[10] ));
  nor002aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  nona22aa1n02x4               g041(.a(new_n132), .b(new_n100), .c(new_n97), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n136), .b(new_n137), .c(new_n98), .out0(\s[11] ));
  aoi013aa1n02x4               g043(.a(new_n134), .b(new_n137), .c(new_n135), .d(new_n98), .o1(new_n139));
  xnrb03aa1n03x5               g044(.a(new_n139), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1d06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano23aa1n03x7               g047(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n143));
  nand23aa1n06x5               g048(.a(new_n143), .b(new_n99), .c(new_n131), .o1(new_n144));
  inv040aa1n02x5               g049(.a(new_n144), .o1(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n130), .c(new_n113), .d(new_n125), .o1(new_n146));
  nano22aa1n03x7               g051(.a(new_n141), .b(new_n135), .c(new_n142), .out0(new_n147));
  tech160nm_fioai012aa1n04x5   g052(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n148));
  oab012aa1n06x5               g053(.a(new_n148), .b(new_n97), .c(new_n100), .out0(new_n149));
  tech160nm_fiao0012aa1n05x5   g054(.a(new_n141), .b(new_n134), .c(new_n142), .o(new_n150));
  aoi012aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n147), .o1(new_n151));
  nanp02aa1n03x5               g056(.a(new_n146), .b(new_n151), .o1(new_n152));
  nor002aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand02aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoi112aa1n02x5               g061(.a(new_n150), .b(new_n156), .c(new_n149), .d(new_n147), .o1(new_n157));
  aoi022aa1n02x5               g062(.a(new_n152), .b(new_n156), .c(new_n146), .d(new_n157), .o1(\s[13] ));
  aoi012aa1n02x5               g063(.a(new_n153), .b(new_n152), .c(new_n154), .o1(new_n159));
  xnrb03aa1n03x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nanp02aa1n04x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1n06x5               g067(.a(new_n153), .b(new_n161), .c(new_n162), .d(new_n154), .out0(new_n163));
  nanp02aa1n02x5               g068(.a(new_n152), .b(new_n163), .o1(new_n164));
  tech160nm_fioai012aa1n03p5x5 g069(.a(new_n162), .b(new_n161), .c(new_n153), .o1(new_n165));
  xnrc02aa1n12x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n164), .c(new_n165), .out0(\s[15] ));
  nona23aa1n02x4               g073(.a(new_n162), .b(new_n154), .c(new_n153), .d(new_n161), .out0(new_n169));
  aoai13aa1n02x7               g074(.a(new_n165), .b(new_n169), .c(new_n146), .d(new_n151), .o1(new_n170));
  nor042aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  tech160nm_fixnrc02aa1n04x5   g076(.a(\b[15] ), .b(\a[16] ), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n170), .d(new_n167), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n171), .b(new_n172), .c(new_n170), .d(new_n167), .o1(new_n174));
  nanb02aa1n03x5               g079(.a(new_n174), .b(new_n173), .out0(\s[16] ));
  aoi012aa1n06x5               g080(.a(new_n130), .b(new_n113), .c(new_n125), .o1(new_n176));
  nor042aa1n06x5               g081(.a(new_n172), .b(new_n166), .o1(new_n177));
  nano32aa1n02x4               g082(.a(new_n176), .b(new_n177), .c(new_n145), .d(new_n163), .out0(new_n178));
  nano22aa1n12x5               g083(.a(new_n144), .b(new_n177), .c(new_n163), .out0(new_n179));
  inv040aa1n02x5               g084(.a(new_n179), .o1(new_n180));
  aoai13aa1n04x5               g085(.a(new_n163), .b(new_n150), .c(new_n149), .d(new_n147), .o1(new_n181));
  aob012aa1n03x5               g086(.a(new_n177), .b(new_n181), .c(new_n165), .out0(new_n182));
  inv000aa1d42x5               g087(.a(\a[16] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\b[15] ), .o1(new_n184));
  oao003aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n171), .carry(new_n185));
  inv000aa1n02x5               g090(.a(new_n185), .o1(new_n186));
  oai112aa1n06x5               g091(.a(new_n182), .b(new_n186), .c(new_n180), .d(new_n176), .o1(new_n187));
  xorc02aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  inv000aa1n02x5               g093(.a(new_n177), .o1(new_n189));
  tech160nm_fiaoi012aa1n05x5   g094(.a(new_n189), .b(new_n181), .c(new_n165), .o1(new_n190));
  norp03aa1n02x5               g095(.a(new_n190), .b(new_n185), .c(new_n188), .o1(new_n191));
  aboi22aa1n03x5               g096(.a(new_n178), .b(new_n191), .c(new_n187), .d(new_n188), .out0(\s[17] ));
  nanp03aa1n03x5               g097(.a(new_n107), .b(\a[1] ), .c(\b[0] ), .o1(new_n193));
  aoi022aa1d24x5               g098(.a(new_n103), .b(new_n102), .c(\a[2] ), .d(\b[1] ), .o1(new_n194));
  inv030aa1n02x5               g099(.a(new_n111), .o1(new_n195));
  oai012aa1n06x5               g100(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .o1(new_n196));
  nor022aa1n04x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  oai112aa1n04x5               g102(.a(new_n197), .b(new_n194), .c(new_n193), .d(new_n106), .o1(new_n198));
  nano23aa1n06x5               g103(.a(new_n114), .b(new_n117), .c(new_n118), .d(new_n115), .out0(new_n199));
  nanb02aa1n03x5               g104(.a(new_n120), .b(new_n121), .out0(new_n200));
  nanb02aa1n06x5               g105(.a(new_n122), .b(new_n123), .out0(new_n201));
  nona22aa1n03x5               g106(.a(new_n199), .b(new_n200), .c(new_n201), .out0(new_n202));
  inv020aa1n03x5               g107(.a(new_n129), .o1(new_n203));
  aoi012aa1n06x5               g108(.a(new_n203), .b(new_n128), .c(new_n126), .o1(new_n204));
  aoai13aa1n09x5               g109(.a(new_n204), .b(new_n202), .c(new_n198), .d(new_n105), .o1(new_n205));
  aoi112aa1n09x5               g110(.a(new_n190), .b(new_n185), .c(new_n205), .d(new_n179), .o1(new_n206));
  oaoi03aa1n03x5               g111(.a(\a[17] ), .b(\b[16] ), .c(new_n206), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g113(.a(\a[17] ), .o1(new_n209));
  inv040aa1n08x5               g114(.a(\a[18] ), .o1(new_n210));
  xroi22aa1d06x4               g115(.a(new_n209), .b(\b[16] ), .c(new_n210), .d(\b[17] ), .out0(new_n211));
  inv030aa1n03x5               g116(.a(new_n211), .o1(new_n212));
  nor042aa1n06x5               g117(.a(\b[16] ), .b(\a[17] ), .o1(new_n213));
  nor042aa1n06x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nand22aa1n04x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  oa0012aa1n02x5               g120(.a(new_n215), .b(new_n214), .c(new_n213), .o(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  oai012aa1n06x5               g122(.a(new_n217), .b(new_n206), .c(new_n212), .o1(new_n218));
  nor042aa1d18x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand42aa1n04x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  norb02aa1n06x4               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  aoi112aa1n02x5               g126(.a(new_n221), .b(new_n216), .c(new_n187), .d(new_n211), .o1(new_n222));
  aoi012aa1n02x5               g127(.a(new_n222), .b(new_n218), .c(new_n221), .o1(\s[19] ));
  xnrc02aa1n02x5               g128(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nand02aa1d16x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n227), .b(new_n219), .c(new_n218), .d(new_n220), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n221), .b(new_n216), .c(new_n187), .d(new_n211), .o1(new_n229));
  nona22aa1n03x5               g134(.a(new_n229), .b(new_n227), .c(new_n219), .out0(new_n230));
  nanp02aa1n03x5               g135(.a(new_n228), .b(new_n230), .o1(\s[20] ));
  aoai13aa1n04x5               g136(.a(new_n186), .b(new_n189), .c(new_n181), .d(new_n165), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n225), .o1(new_n233));
  nano32aa1n03x7               g138(.a(new_n212), .b(new_n226), .c(new_n221), .d(new_n233), .out0(new_n234));
  aoai13aa1n02x5               g139(.a(new_n234), .b(new_n232), .c(new_n205), .d(new_n179), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n234), .o1(new_n236));
  nanb03aa1n12x5               g141(.a(new_n225), .b(new_n226), .c(new_n220), .out0(new_n237));
  oai122aa1n12x5               g142(.a(new_n215), .b(new_n214), .c(new_n213), .d(\b[18] ), .e(\a[19] ), .o1(new_n238));
  aoi012aa1n06x5               g143(.a(new_n225), .b(new_n219), .c(new_n226), .o1(new_n239));
  oai012aa1d24x5               g144(.a(new_n239), .b(new_n238), .c(new_n237), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  oai012aa1n04x7               g146(.a(new_n241), .b(new_n206), .c(new_n236), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[20] ), .b(\a[21] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nano22aa1n02x5               g149(.a(new_n225), .b(new_n220), .c(new_n226), .out0(new_n245));
  oai012aa1n02x5               g150(.a(new_n215), .b(\b[18] ), .c(\a[19] ), .o1(new_n246));
  oab012aa1n02x5               g151(.a(new_n246), .b(new_n213), .c(new_n214), .out0(new_n247));
  inv020aa1n03x5               g152(.a(new_n239), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(new_n244), .b(new_n248), .c(new_n247), .d(new_n245), .o1(new_n249));
  aoi022aa1n02x5               g154(.a(new_n242), .b(new_n244), .c(new_n235), .d(new_n249), .o1(\s[21] ));
  nor042aa1n12x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  tech160nm_fixnrc02aa1n04x5   g156(.a(\b[21] ), .b(\a[22] ), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n251), .c(new_n242), .d(new_n244), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n244), .b(new_n240), .c(new_n187), .d(new_n234), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n251), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n253), .b(new_n255), .o1(\s[22] ));
  nor042aa1n06x5               g161(.a(new_n252), .b(new_n243), .o1(new_n257));
  nano32aa1n03x7               g162(.a(new_n227), .b(new_n211), .c(new_n257), .d(new_n221), .out0(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n232), .c(new_n205), .d(new_n179), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n258), .o1(new_n260));
  inv040aa1d32x5               g165(.a(\a[22] ), .o1(new_n261));
  inv040aa1d32x5               g166(.a(\b[21] ), .o1(new_n262));
  oao003aa1n02x5               g167(.a(new_n261), .b(new_n262), .c(new_n251), .carry(new_n263));
  aoi012aa1d24x5               g168(.a(new_n263), .b(new_n240), .c(new_n257), .o1(new_n264));
  tech160nm_fioai012aa1n05x5   g169(.a(new_n264), .b(new_n206), .c(new_n260), .o1(new_n265));
  xorc02aa1n12x5               g170(.a(\a[23] ), .b(\b[22] ), .out0(new_n266));
  aoi112aa1n02x5               g171(.a(new_n266), .b(new_n263), .c(new_n240), .d(new_n257), .o1(new_n267));
  aoi022aa1n02x5               g172(.a(new_n265), .b(new_n266), .c(new_n259), .d(new_n267), .o1(\s[23] ));
  norp02aa1n02x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[23] ), .b(\a[24] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n269), .c(new_n265), .d(new_n266), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n264), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n266), .b(new_n272), .c(new_n187), .d(new_n258), .o1(new_n273));
  nona22aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n269), .out0(new_n274));
  nanp02aa1n03x5               g179(.a(new_n271), .b(new_n274), .o1(\s[24] ));
  norb02aa1n06x4               g180(.a(new_n266), .b(new_n270), .out0(new_n276));
  nand02aa1d04x5               g181(.a(new_n258), .b(new_n276), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  oai012aa1n02x5               g183(.a(new_n278), .b(new_n178), .c(new_n232), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n257), .b(new_n248), .c(new_n247), .d(new_n245), .o1(new_n280));
  inv040aa1n02x5               g185(.a(new_n263), .o1(new_n281));
  inv000aa1n03x5               g186(.a(new_n276), .o1(new_n282));
  oai022aa1n02x5               g187(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n283));
  aob012aa1n02x5               g188(.a(new_n283), .b(\b[23] ), .c(\a[24] ), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n282), .c(new_n280), .d(new_n281), .o1(new_n285));
  inv000aa1n02x5               g190(.a(new_n285), .o1(new_n286));
  tech160nm_fioai012aa1n05x5   g191(.a(new_n286), .b(new_n206), .c(new_n277), .o1(new_n287));
  tech160nm_fixorc02aa1n03p5x5 g192(.a(\a[25] ), .b(\b[24] ), .out0(new_n288));
  aoai13aa1n06x5               g193(.a(new_n276), .b(new_n263), .c(new_n240), .d(new_n257), .o1(new_n289));
  nano22aa1n02x4               g194(.a(new_n288), .b(new_n289), .c(new_n284), .out0(new_n290));
  aoi022aa1n02x5               g195(.a(new_n287), .b(new_n288), .c(new_n279), .d(new_n290), .o1(\s[25] ));
  nor002aa1n02x5               g196(.a(\b[24] ), .b(\a[25] ), .o1(new_n292));
  xnrc02aa1n12x5               g197(.a(\b[25] ), .b(\a[26] ), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n292), .c(new_n287), .d(new_n288), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n288), .b(new_n285), .c(new_n187), .d(new_n278), .o1(new_n295));
  nona22aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n292), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[26] ));
  norb02aa1n06x5               g202(.a(new_n288), .b(new_n293), .out0(new_n298));
  nano32aa1n03x7               g203(.a(new_n236), .b(new_n298), .c(new_n257), .d(new_n276), .out0(new_n299));
  aoai13aa1n04x5               g204(.a(new_n299), .b(new_n232), .c(new_n205), .d(new_n179), .o1(new_n300));
  inv000aa1d42x5               g205(.a(\a[26] ), .o1(new_n301));
  inv000aa1d42x5               g206(.a(\b[25] ), .o1(new_n302));
  oao003aa1n02x5               g207(.a(new_n301), .b(new_n302), .c(new_n292), .carry(new_n303));
  aoi012aa1n09x5               g208(.a(new_n303), .b(new_n285), .c(new_n298), .o1(new_n304));
  xorc02aa1n12x5               g209(.a(\a[27] ), .b(\b[26] ), .out0(new_n305));
  xnbna2aa1n03x5               g210(.a(new_n305), .b(new_n300), .c(new_n304), .out0(\s[27] ));
  inv000aa1d42x5               g211(.a(new_n298), .o1(new_n307));
  nona22aa1n03x5               g212(.a(new_n258), .b(new_n282), .c(new_n307), .out0(new_n308));
  oai012aa1n06x5               g213(.a(new_n304), .b(new_n206), .c(new_n308), .o1(new_n309));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  norp02aa1n02x5               g215(.a(\b[27] ), .b(\a[28] ), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(\b[27] ), .b(\a[28] ), .o1(new_n312));
  nanb02aa1n02x5               g217(.a(new_n311), .b(new_n312), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n310), .c(new_n309), .d(new_n305), .o1(new_n314));
  inv000aa1n02x5               g219(.a(new_n303), .o1(new_n315));
  aoai13aa1n04x5               g220(.a(new_n315), .b(new_n307), .c(new_n289), .d(new_n284), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n305), .b(new_n316), .c(new_n187), .d(new_n299), .o1(new_n317));
  nona22aa1n02x4               g222(.a(new_n317), .b(new_n313), .c(new_n310), .out0(new_n318));
  nanp02aa1n03x5               g223(.a(new_n314), .b(new_n318), .o1(\s[28] ));
  norb02aa1n02x5               g224(.a(new_n305), .b(new_n313), .out0(new_n320));
  aoai13aa1n06x5               g225(.a(new_n320), .b(new_n316), .c(new_n187), .d(new_n299), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n320), .o1(new_n322));
  aoi012aa1n02x5               g227(.a(new_n311), .b(new_n310), .c(new_n312), .o1(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n322), .c(new_n300), .d(new_n304), .o1(new_n324));
  tech160nm_fixorc02aa1n03p5x5 g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n02x7               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n06x5               g233(.a(new_n313), .b(new_n325), .c(new_n305), .out0(new_n329));
  nanb02aa1n03x5               g234(.a(new_n329), .b(new_n309), .out0(new_n330));
  oao003aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n323), .carry(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n329), .c(new_n300), .d(new_n304), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .out0(new_n333));
  norb02aa1n02x5               g238(.a(new_n331), .b(new_n333), .out0(new_n334));
  aoi022aa1n03x5               g239(.a(new_n332), .b(new_n333), .c(new_n330), .d(new_n334), .o1(\s[30] ));
  inv000aa1d42x5               g240(.a(new_n325), .o1(new_n336));
  inv000aa1n02x5               g241(.a(new_n333), .o1(new_n337));
  nona32aa1n03x5               g242(.a(new_n309), .b(new_n337), .c(new_n336), .d(new_n322), .out0(new_n338));
  xorc02aa1n02x5               g243(.a(\a[31] ), .b(\b[30] ), .out0(new_n339));
  oao003aa1n02x5               g244(.a(\a[30] ), .b(\b[29] ), .c(new_n331), .carry(new_n340));
  norb02aa1n02x5               g245(.a(new_n340), .b(new_n339), .out0(new_n341));
  nona22aa1n02x4               g246(.a(new_n320), .b(new_n336), .c(new_n337), .out0(new_n342));
  aoai13aa1n03x5               g247(.a(new_n340), .b(new_n342), .c(new_n300), .d(new_n304), .o1(new_n343));
  aoi022aa1n03x5               g248(.a(new_n343), .b(new_n339), .c(new_n338), .d(new_n341), .o1(\s[31] ));
  oai012aa1n02x5               g249(.a(new_n107), .b(new_n106), .c(new_n108), .o1(new_n345));
  nona22aa1n02x4               g250(.a(new_n345), .b(new_n195), .c(new_n104), .out0(new_n346));
  oai122aa1n02x7               g251(.a(new_n107), .b(new_n195), .c(new_n104), .d(new_n108), .e(new_n106), .o1(new_n347));
  nanp02aa1n02x5               g252(.a(new_n347), .b(new_n346), .o1(\s[3] ));
  xnrc02aa1n02x5               g253(.a(\b[3] ), .b(\a[4] ), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n346), .c(new_n111), .out0(\s[4] ));
  nanb03aa1n02x5               g255(.a(new_n119), .b(new_n198), .c(new_n105), .out0(new_n351));
  aob012aa1n02x5               g256(.a(new_n351), .b(new_n119), .c(new_n113), .out0(\s[5] ));
  xnbna2aa1n03x5               g257(.a(new_n116), .b(new_n351), .c(new_n118), .out0(\s[6] ));
  aoai13aa1n02x5               g258(.a(new_n115), .b(new_n114), .c(new_n351), .d(new_n118), .o1(new_n354));
  aoi012aa1n03x5               g259(.a(new_n116), .b(new_n351), .c(new_n118), .o1(new_n355));
  nano23aa1n02x4               g260(.a(new_n355), .b(new_n122), .c(new_n115), .d(new_n123), .out0(new_n356));
  aoi012aa1n02x5               g261(.a(new_n356), .b(new_n201), .c(new_n354), .o1(\s[7] ));
  oai012aa1n02x5               g262(.a(new_n200), .b(new_n356), .c(new_n122), .o1(new_n358));
  orn003aa1n02x5               g263(.a(new_n356), .b(new_n200), .c(new_n122), .o(new_n359));
  nanp02aa1n02x5               g264(.a(new_n359), .b(new_n358), .o1(\s[8] ));
  nanp02aa1n02x5               g265(.a(new_n113), .b(new_n125), .o1(new_n361));
  aoi112aa1n02x5               g266(.a(new_n203), .b(new_n131), .c(new_n128), .d(new_n126), .o1(new_n362));
  aoi022aa1n02x5               g267(.a(new_n205), .b(new_n131), .c(new_n361), .d(new_n362), .o1(\s[9] ));
endmodule


