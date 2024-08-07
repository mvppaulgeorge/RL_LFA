// Benchmark "adder" written by ABC on Thu Jul 18 04:26:55 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n328, new_n329, new_n332, new_n334,
    new_n335, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[8] ), .o1(new_n99));
  orn002aa1n02x5               g004(.a(\a[8] ), .b(\b[7] ), .o(new_n100));
  nand42aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanp02aa1n06x5               g006(.a(new_n100), .b(new_n101), .o1(new_n102));
  nor022aa1n06x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanb02aa1n03x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  nor002aa1d32x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nand42aa1d28x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  norb02aa1n03x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  xorc02aa1n02x5               g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n108), .c(new_n102), .d(new_n105), .out0(new_n110));
  nand42aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  tech160nm_fixorc02aa1n03p5x5 g016(.a(\a[3] ), .b(\b[2] ), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[2] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[1] ), .o1(new_n114));
  nand02aa1n04x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nand42aa1n06x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  aob012aa1n12x5               g021(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n115), .o1(new_n118));
  oa0022aa1n06x5               g023(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n119));
  inv040aa1n02x5               g024(.a(new_n119), .o1(new_n120));
  aoai13aa1n04x5               g025(.a(new_n111), .b(new_n120), .c(new_n118), .d(new_n112), .o1(new_n121));
  nand42aa1n03x5               g026(.a(new_n103), .b(new_n101), .o1(new_n122));
  nor042aa1n09x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  oai012aa1d24x5               g028(.a(new_n107), .b(new_n123), .c(new_n106), .o1(new_n124));
  norp03aa1n06x5               g029(.a(new_n124), .b(new_n102), .c(new_n105), .o1(new_n125));
  nano22aa1n06x5               g030(.a(new_n125), .b(new_n100), .c(new_n122), .out0(new_n126));
  tech160nm_fioai012aa1n04x5   g031(.a(new_n126), .b(new_n121), .c(new_n110), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(new_n98), .b(new_n99), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nor002aa1n16x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand22aa1n03x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oai022aa1n12x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  oaib12aa1n18x5               g038(.a(new_n133), .b(new_n97), .c(\b[9] ), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  xroi22aa1d04x5               g040(.a(new_n97), .b(\b[9] ), .c(new_n99), .d(\a[9] ), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n132), .b(new_n135), .c(new_n127), .d(new_n136), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n135), .b(new_n132), .c(new_n127), .d(new_n136), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n130), .o1(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand22aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n137), .c(new_n140), .out0(\s[12] ));
  nor022aa1n02x5               g049(.a(\b[7] ), .b(\a[8] ), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n104), .b(new_n101), .c(new_n145), .d(new_n103), .out0(new_n146));
  tech160nm_fixnrc02aa1n05x5   g051(.a(\b[4] ), .b(\a[5] ), .out0(new_n147));
  norb02aa1n03x5               g052(.a(new_n108), .b(new_n147), .out0(new_n148));
  inv000aa1n02x5               g053(.a(new_n111), .o1(new_n149));
  xnrc02aa1n12x5               g054(.a(\b[2] ), .b(\a[3] ), .out0(new_n150));
  aoai13aa1n12x5               g055(.a(new_n119), .b(new_n150), .c(new_n117), .d(new_n115), .o1(new_n151));
  nona23aa1n09x5               g056(.a(new_n151), .b(new_n148), .c(new_n146), .d(new_n149), .out0(new_n152));
  xorc02aa1n02x5               g057(.a(\a[10] ), .b(\b[9] ), .out0(new_n153));
  xorc02aa1n02x5               g058(.a(\a[9] ), .b(\b[8] ), .out0(new_n154));
  nano23aa1n06x5               g059(.a(new_n130), .b(new_n141), .c(new_n142), .d(new_n131), .out0(new_n155));
  nanp03aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n154), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n141), .b(new_n130), .c(new_n142), .o1(new_n157));
  aobi12aa1n06x5               g062(.a(new_n157), .b(new_n155), .c(new_n135), .out0(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n156), .c(new_n152), .d(new_n126), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n04x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n12x5               g069(.a(\b[14] ), .b(\a[15] ), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  nor002aa1d32x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand42aa1n08x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nona23aa1d18x5               g073(.a(new_n168), .b(new_n162), .c(new_n161), .d(new_n167), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  oai012aa1n12x5               g075(.a(new_n168), .b(new_n167), .c(new_n161), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  aoai13aa1n06x5               g077(.a(new_n166), .b(new_n172), .c(new_n159), .d(new_n170), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n166), .b(new_n172), .c(new_n159), .d(new_n170), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(\s[15] ));
  nor042aa1d18x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  xnrc02aa1n12x5               g081(.a(\b[15] ), .b(\a[16] ), .out0(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  nona22aa1n02x4               g083(.a(new_n173), .b(new_n178), .c(new_n176), .out0(new_n179));
  inv000aa1n02x5               g084(.a(new_n176), .o1(new_n180));
  tech160nm_fiaoi012aa1n03p5x5 g085(.a(new_n177), .b(new_n173), .c(new_n180), .o1(new_n181));
  norb02aa1n03x4               g086(.a(new_n179), .b(new_n181), .out0(\s[16] ));
  nor043aa1n03x5               g087(.a(new_n169), .b(new_n177), .c(new_n165), .o1(new_n183));
  nand23aa1n03x5               g088(.a(new_n183), .b(new_n136), .c(new_n155), .o1(new_n184));
  nona23aa1n02x4               g089(.a(new_n142), .b(new_n131), .c(new_n130), .d(new_n141), .out0(new_n185));
  oai012aa1n03x5               g090(.a(new_n157), .b(new_n185), .c(new_n134), .o1(new_n186));
  oao003aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .c(new_n180), .carry(new_n187));
  oai013aa1n03x4               g092(.a(new_n187), .b(new_n177), .c(new_n165), .d(new_n171), .o1(new_n188));
  aoi012aa1n06x5               g093(.a(new_n188), .b(new_n186), .c(new_n183), .o1(new_n189));
  aoai13aa1n12x5               g094(.a(new_n189), .b(new_n184), .c(new_n152), .d(new_n126), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d48x5               g097(.a(\b[16] ), .o1(new_n193));
  tech160nm_fioaoi03aa1n03p5x5 g098(.a(new_n192), .b(new_n193), .c(new_n190), .o1(new_n194));
  xnrb03aa1n03x5               g099(.a(new_n194), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n03x5               g100(.a(new_n193), .b(new_n192), .o1(new_n196));
  and002aa1n02x5               g101(.a(\b[16] ), .b(\a[17] ), .o(new_n197));
  nor022aa1n16x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nand02aa1d08x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nano23aa1n06x5               g104(.a(new_n198), .b(new_n197), .c(new_n196), .d(new_n199), .out0(new_n200));
  aoai13aa1n12x5               g105(.a(new_n199), .b(new_n198), .c(new_n192), .d(new_n193), .o1(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand22aa1n04x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanb02aa1n09x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n202), .c(new_n190), .d(new_n200), .o1(new_n207));
  aoi112aa1n02x7               g112(.a(new_n206), .b(new_n202), .c(new_n190), .d(new_n200), .o1(new_n208));
  norb02aa1n03x4               g113(.a(new_n207), .b(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g115(.a(\b[19] ), .o1(new_n211));
  nanb02aa1n12x5               g116(.a(\a[20] ), .b(new_n211), .out0(new_n212));
  nand22aa1n09x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nand42aa1n06x5               g118(.a(new_n212), .b(new_n213), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nona22aa1n03x5               g120(.a(new_n207), .b(new_n215), .c(new_n203), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n203), .o1(new_n217));
  aoi012aa1n03x5               g122(.a(new_n214), .b(new_n207), .c(new_n217), .o1(new_n218));
  norb02aa1n02x7               g123(.a(new_n216), .b(new_n218), .out0(\s[20] ));
  nor042aa1n04x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nona23aa1n12x5               g125(.a(new_n213), .b(new_n204), .c(new_n203), .d(new_n220), .out0(new_n221));
  inv040aa1n04x5               g126(.a(new_n221), .o1(new_n222));
  nanp02aa1n03x5               g127(.a(new_n222), .b(new_n200), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  nanp02aa1n02x5               g129(.a(new_n203), .b(new_n213), .o1(new_n225));
  oai112aa1n06x5               g130(.a(new_n225), .b(new_n212), .c(new_n221), .d(new_n201), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n226), .c(new_n190), .d(new_n224), .o1(new_n229));
  aoi112aa1n02x7               g134(.a(new_n228), .b(new_n226), .c(new_n190), .d(new_n224), .o1(new_n230));
  norb02aa1n03x4               g135(.a(new_n229), .b(new_n230), .out0(\s[21] ));
  nor042aa1n03x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  nona22aa1n03x5               g139(.a(new_n229), .b(new_n234), .c(new_n232), .out0(new_n235));
  inv020aa1n02x5               g140(.a(new_n232), .o1(new_n236));
  aoi012aa1n03x5               g141(.a(new_n233), .b(new_n229), .c(new_n236), .o1(new_n237));
  norb02aa1n02x7               g142(.a(new_n235), .b(new_n237), .out0(\s[22] ));
  nor042aa1n06x5               g143(.a(new_n233), .b(new_n227), .o1(new_n239));
  tech160nm_fioaoi03aa1n03p5x5 g144(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n226), .c(new_n239), .o1(new_n241));
  inv000aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  nand23aa1n06x5               g147(.a(new_n222), .b(new_n239), .c(new_n200), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n190), .d(new_n244), .o1(new_n246));
  aoi112aa1n02x7               g151(.a(new_n242), .b(new_n245), .c(new_n190), .d(new_n244), .o1(new_n247));
  norb02aa1n03x4               g152(.a(new_n246), .b(new_n247), .out0(\s[23] ));
  norp02aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  xorc02aa1n12x5               g154(.a(\a[24] ), .b(\b[23] ), .out0(new_n250));
  nona22aa1n03x5               g155(.a(new_n246), .b(new_n250), .c(new_n249), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n249), .o1(new_n252));
  aobi12aa1n03x5               g157(.a(new_n250), .b(new_n246), .c(new_n252), .out0(new_n253));
  norb02aa1n02x7               g158(.a(new_n251), .b(new_n253), .out0(\s[24] ));
  nano32aa1n02x5               g159(.a(new_n223), .b(new_n250), .c(new_n239), .d(new_n245), .out0(new_n255));
  xnrc02aa1n02x5               g160(.a(\b[22] ), .b(\a[23] ), .out0(new_n256));
  norb02aa1n02x5               g161(.a(new_n250), .b(new_n256), .out0(new_n257));
  norp02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n259));
  nanp03aa1n03x5               g164(.a(new_n240), .b(new_n245), .c(new_n250), .o1(new_n260));
  nona22aa1n02x4               g165(.a(new_n260), .b(new_n259), .c(new_n258), .out0(new_n261));
  aoi013aa1n02x4               g166(.a(new_n261), .b(new_n226), .c(new_n239), .d(new_n257), .o1(new_n262));
  inv040aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  tech160nm_fixorc02aa1n03p5x5 g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n190), .d(new_n255), .o1(new_n265));
  aoi112aa1n02x7               g170(.a(new_n264), .b(new_n263), .c(new_n190), .d(new_n255), .o1(new_n266));
  norb02aa1n03x4               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  tech160nm_fixorc02aa1n03p5x5 g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  nona22aa1n03x5               g174(.a(new_n265), .b(new_n269), .c(new_n268), .out0(new_n270));
  inv000aa1n03x5               g175(.a(new_n268), .o1(new_n271));
  aobi12aa1n03x5               g176(.a(new_n269), .b(new_n265), .c(new_n271), .out0(new_n272));
  norb02aa1n02x7               g177(.a(new_n270), .b(new_n272), .out0(\s[26] ));
  nona22aa1n02x4               g178(.a(new_n170), .b(new_n177), .c(new_n165), .out0(new_n274));
  norp02aa1n02x5               g179(.a(new_n274), .b(new_n156), .o1(new_n275));
  oabi12aa1n02x5               g180(.a(new_n188), .b(new_n158), .c(new_n274), .out0(new_n276));
  and002aa1n06x5               g181(.a(new_n269), .b(new_n264), .o(new_n277));
  nano22aa1n12x5               g182(.a(new_n243), .b(new_n277), .c(new_n257), .out0(new_n278));
  aoai13aa1n06x5               g183(.a(new_n278), .b(new_n276), .c(new_n127), .d(new_n275), .o1(new_n279));
  nor043aa1n03x5               g184(.a(new_n201), .b(new_n205), .c(new_n214), .o1(new_n280));
  nano22aa1n03x7               g185(.a(new_n280), .b(new_n212), .c(new_n225), .out0(new_n281));
  nano22aa1n06x5               g186(.a(new_n281), .b(new_n239), .c(new_n257), .out0(new_n282));
  oao003aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .c(new_n271), .carry(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  oaoi13aa1n09x5               g189(.a(new_n284), .b(new_n277), .c(new_n282), .d(new_n261), .o1(new_n285));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norb02aa1n02x5               g192(.a(new_n287), .b(new_n286), .out0(new_n288));
  xnbna2aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n279), .out0(\s[27] ));
  inv000aa1n06x5               g194(.a(new_n286), .o1(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[27] ), .b(\a[28] ), .out0(new_n291));
  inv020aa1n02x5               g196(.a(new_n261), .o1(new_n292));
  nanp02aa1n02x5               g197(.a(new_n250), .b(new_n245), .o1(new_n293));
  nona32aa1n02x4               g198(.a(new_n226), .b(new_n293), .c(new_n233), .d(new_n227), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n277), .o1(new_n295));
  aoai13aa1n12x5               g200(.a(new_n283), .b(new_n295), .c(new_n294), .d(new_n292), .o1(new_n296));
  aoai13aa1n03x5               g201(.a(new_n287), .b(new_n296), .c(new_n190), .d(new_n278), .o1(new_n297));
  aoi012aa1n03x5               g202(.a(new_n291), .b(new_n297), .c(new_n290), .o1(new_n298));
  aoi022aa1n02x7               g203(.a(new_n285), .b(new_n279), .c(\a[27] ), .d(\b[26] ), .o1(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n290), .c(new_n291), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[28] ));
  nano22aa1n02x4               g206(.a(new_n291), .b(new_n290), .c(new_n287), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n296), .c(new_n190), .d(new_n278), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .c(new_n290), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .out0(new_n305));
  aoi012aa1n03x5               g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n02x5               g211(.a(new_n302), .b(new_n285), .c(new_n279), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  nor002aa1n02x5               g213(.a(new_n306), .b(new_n308), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g215(.a(new_n288), .b(new_n305), .c(new_n291), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n296), .c(new_n190), .d(new_n278), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .c(new_n304), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[29] ), .b(\a[30] ), .out0(new_n314));
  aoi012aa1n03x5               g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  aobi12aa1n03x5               g220(.a(new_n311), .b(new_n285), .c(new_n279), .out0(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n314), .out0(new_n317));
  nor002aa1n02x5               g222(.a(new_n315), .b(new_n317), .o1(\s[30] ));
  xnrc02aa1n02x5               g223(.a(\b[30] ), .b(\a[31] ), .out0(new_n319));
  norb03aa1n02x5               g224(.a(new_n302), .b(new_n314), .c(new_n305), .out0(new_n320));
  aobi12aa1n03x5               g225(.a(new_n320), .b(new_n285), .c(new_n279), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n313), .carry(new_n322));
  nano22aa1n03x5               g227(.a(new_n321), .b(new_n319), .c(new_n322), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n320), .b(new_n296), .c(new_n190), .d(new_n278), .o1(new_n324));
  aoi012aa1n03x5               g229(.a(new_n319), .b(new_n324), .c(new_n322), .o1(new_n325));
  nor002aa1n02x5               g230(.a(new_n325), .b(new_n323), .o1(\s[31] ));
  xnbna2aa1n03x5               g231(.a(new_n112), .b(new_n117), .c(new_n115), .out0(\s[3] ));
  orn002aa1n02x5               g232(.a(\a[3] ), .b(\b[2] ), .o(new_n328));
  aoai13aa1n02x5               g233(.a(new_n328), .b(new_n150), .c(new_n117), .d(new_n115), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g235(.a(new_n147), .b(new_n151), .c(new_n111), .out0(\s[5] ));
  oaoi03aa1n02x5               g236(.a(\a[5] ), .b(\b[4] ), .c(new_n121), .o1(new_n332));
  xorb03aa1n02x5               g237(.a(new_n332), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g238(.a(new_n124), .o1(new_n334));
  aoi013aa1n02x4               g239(.a(new_n334), .b(new_n151), .c(new_n111), .d(new_n148), .o1(new_n335));
  xnrb03aa1n02x5               g240(.a(new_n335), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g241(.a(\a[7] ), .b(\b[6] ), .c(new_n335), .o1(new_n337));
  xorb03aa1n02x5               g242(.a(new_n337), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g243(.a(new_n154), .b(new_n152), .c(new_n126), .out0(\s[9] ));
endmodule


