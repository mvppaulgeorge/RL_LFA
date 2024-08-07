// Benchmark "adder" written by ABC on Wed Jul 17 16:51:47 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n331,
    new_n332, new_n334, new_n336, new_n338, new_n339, new_n342;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  nor042aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d08x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nand02aa1d08x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi012aa1n12x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  tech160nm_fioai012aa1n05x5   g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n06x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand02aa1d24x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1n12x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1n08x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nand02aa1d28x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor002aa1d32x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n12x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xorc02aa1n12x5               g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  norb03aa1n03x5               g024(.a(new_n119), .b(new_n115), .c(new_n118), .out0(new_n120));
  inv000aa1d42x5               g025(.a(new_n111), .o1(new_n121));
  inv000aa1d42x5               g026(.a(new_n112), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n113), .o1(new_n123));
  nor042aa1n12x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  aoai13aa1n12x5               g029(.a(new_n114), .b(new_n117), .c(new_n124), .d(new_n116), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n121), .b(new_n122), .c(new_n125), .d(new_n123), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n120), .d(new_n110), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n97), .b(new_n128), .c(new_n99), .out0(\s[10] ));
  oaih22aa1n06x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n130), .b(new_n128), .out0(new_n131));
  nanp02aa1n04x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nand42aa1n20x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n06x4               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  aoi012aa1n02x5               g040(.a(new_n135), .b(new_n131), .c(new_n132), .o1(new_n136));
  nano22aa1n02x4               g041(.a(new_n134), .b(new_n132), .c(new_n133), .out0(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n131), .c(new_n137), .o1(\s[11] ));
  tech160nm_fiaoi012aa1n05x5   g043(.a(new_n134), .b(new_n131), .c(new_n137), .o1(new_n139));
  nor002aa1d32x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  nand42aa1n20x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n139), .b(new_n142), .c(new_n141), .out0(\s[12] ));
  nano23aa1n06x5               g048(.a(new_n140), .b(new_n134), .c(new_n142), .d(new_n133), .out0(new_n144));
  and003aa1n02x5               g049(.a(new_n144), .b(new_n127), .c(new_n97), .o(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n126), .c(new_n120), .d(new_n110), .o1(new_n146));
  oa0012aa1n03x5               g051(.a(new_n142), .b(new_n140), .c(new_n134), .o(new_n147));
  nano22aa1n12x5               g052(.a(new_n140), .b(new_n132), .c(new_n142), .out0(new_n148));
  aoi013aa1n06x4               g053(.a(new_n147), .b(new_n148), .c(new_n135), .d(new_n130), .o1(new_n149));
  nor042aa1d18x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nand42aa1d28x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  norb02aa1n02x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n152), .b(new_n146), .c(new_n149), .out0(\s[13] ));
  nanp02aa1n02x5               g058(.a(new_n146), .b(new_n149), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n150), .b(new_n154), .c(new_n152), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv020aa1n04x5               g061(.a(new_n103), .o1(new_n157));
  nano23aa1n09x5               g062(.a(new_n104), .b(new_n106), .c(new_n107), .d(new_n105), .out0(new_n158));
  aobi12aa1n12x5               g063(.a(new_n109), .b(new_n158), .c(new_n157), .out0(new_n159));
  nanb02aa1n03x5               g064(.a(new_n111), .b(new_n112), .out0(new_n160));
  norb02aa1n03x5               g065(.a(new_n114), .b(new_n113), .out0(new_n161));
  nona23aa1n12x5               g066(.a(new_n119), .b(new_n161), .c(new_n160), .d(new_n118), .out0(new_n162));
  nanp02aa1n06x5               g067(.a(new_n125), .b(new_n123), .o1(new_n163));
  tech160nm_fiaoi012aa1n03p5x5 g068(.a(new_n111), .b(new_n163), .c(new_n112), .o1(new_n164));
  oaih12aa1n12x5               g069(.a(new_n164), .b(new_n159), .c(new_n162), .o1(new_n165));
  nand03aa1n06x5               g070(.a(new_n148), .b(new_n135), .c(new_n130), .o1(new_n166));
  nanb02aa1n06x5               g071(.a(new_n147), .b(new_n166), .out0(new_n167));
  nor042aa1n06x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand42aa1d28x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nano23aa1d15x5               g074(.a(new_n150), .b(new_n168), .c(new_n169), .d(new_n151), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n167), .c(new_n165), .d(new_n145), .o1(new_n171));
  aoi012aa1n02x5               g076(.a(new_n168), .b(new_n150), .c(new_n169), .o1(new_n172));
  nor042aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n16x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  xnbna2aa1n03x5               g080(.a(new_n175), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  nanp02aa1n06x5               g081(.a(new_n171), .b(new_n172), .o1(new_n177));
  nor042aa1d18x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nand42aa1d28x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n173), .c(new_n177), .d(new_n174), .o1(new_n181));
  nanp02aa1n02x5               g086(.a(new_n177), .b(new_n175), .o1(new_n182));
  nona22aa1n02x4               g087(.a(new_n182), .b(new_n180), .c(new_n173), .out0(new_n183));
  nanp02aa1n03x5               g088(.a(new_n183), .b(new_n181), .o1(\s[16] ));
  nano23aa1n09x5               g089(.a(new_n173), .b(new_n178), .c(new_n179), .d(new_n174), .out0(new_n185));
  nand22aa1n09x5               g090(.a(new_n185), .b(new_n170), .o1(new_n186));
  nano32aa1d12x5               g091(.a(new_n186), .b(new_n144), .c(new_n127), .d(new_n97), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n126), .c(new_n120), .d(new_n110), .o1(new_n188));
  inv000aa1n02x5               g093(.a(new_n186), .o1(new_n189));
  inv000aa1n02x5               g094(.a(new_n173), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n178), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n179), .o1(new_n192));
  aoai13aa1n04x5               g097(.a(new_n174), .b(new_n168), .c(new_n150), .d(new_n169), .o1(new_n193));
  aoai13aa1n06x5               g098(.a(new_n191), .b(new_n192), .c(new_n193), .d(new_n190), .o1(new_n194));
  aoi012aa1n06x5               g099(.a(new_n194), .b(new_n167), .c(new_n189), .o1(new_n195));
  nand02aa1d06x5               g100(.a(new_n188), .b(new_n195), .o1(new_n196));
  nor042aa1n12x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  nand42aa1n16x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoi112aa1n02x5               g104(.a(new_n199), .b(new_n194), .c(new_n167), .d(new_n189), .o1(new_n200));
  aoi022aa1n02x5               g105(.a(new_n196), .b(new_n199), .c(new_n188), .d(new_n200), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g106(.a(new_n197), .b(new_n196), .c(new_n199), .o1(new_n202));
  xnrb03aa1n03x5               g107(.a(new_n202), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  oabi12aa1n06x5               g108(.a(new_n194), .b(new_n186), .c(new_n149), .out0(new_n204));
  nor042aa1n09x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nano23aa1d15x5               g111(.a(new_n197), .b(new_n205), .c(new_n206), .d(new_n198), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n204), .c(new_n165), .d(new_n187), .o1(new_n208));
  oa0012aa1n02x5               g113(.a(new_n206), .b(new_n205), .c(new_n197), .o(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nor042aa1n04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand02aa1n08x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  norb02aa1n03x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n208), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g120(.a(new_n208), .b(new_n210), .o1(new_n216));
  nor022aa1n08x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand22aa1n12x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanb02aa1n12x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n211), .c(new_n216), .d(new_n212), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n213), .b(new_n209), .c(new_n196), .d(new_n207), .o1(new_n221));
  nona22aa1n02x5               g126(.a(new_n221), .b(new_n219), .c(new_n211), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n220), .b(new_n222), .o1(\s[20] ));
  nanb03aa1n12x5               g128(.a(new_n217), .b(new_n218), .c(new_n212), .out0(new_n224));
  oai122aa1n12x5               g129(.a(new_n206), .b(new_n205), .c(new_n197), .d(\b[18] ), .e(\a[19] ), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n217), .o1(new_n226));
  aob012aa1n06x5               g131(.a(new_n226), .b(new_n211), .c(new_n218), .out0(new_n227));
  oabi12aa1n18x5               g132(.a(new_n227), .b(new_n225), .c(new_n224), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  nanb03aa1n06x5               g134(.a(new_n219), .b(new_n207), .c(new_n213), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n229), .b(new_n230), .c(new_n188), .d(new_n195), .o1(new_n231));
  nor042aa1n06x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand02aa1n06x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  aoib12aa1n02x5               g139(.a(new_n234), .b(new_n196), .c(new_n230), .out0(new_n235));
  aoi022aa1n02x5               g140(.a(new_n235), .b(new_n229), .c(new_n231), .d(new_n234), .o1(\s[21] ));
  nor042aa1n03x5               g141(.a(\b[21] ), .b(\a[22] ), .o1(new_n237));
  nanp02aa1n04x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nanb02aa1n02x5               g143(.a(new_n237), .b(new_n238), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n232), .c(new_n231), .d(new_n234), .o1(new_n240));
  aoi112aa1n03x5               g145(.a(new_n232), .b(new_n239), .c(new_n231), .d(new_n233), .o1(new_n241));
  nanb02aa1n03x5               g146(.a(new_n241), .b(new_n240), .out0(\s[22] ));
  nano23aa1d15x5               g147(.a(new_n232), .b(new_n237), .c(new_n238), .d(new_n233), .out0(new_n243));
  nano32aa1n02x4               g148(.a(new_n219), .b(new_n243), .c(new_n207), .d(new_n213), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n204), .c(new_n165), .d(new_n187), .o1(new_n245));
  nano22aa1n03x7               g150(.a(new_n217), .b(new_n212), .c(new_n218), .out0(new_n246));
  oai012aa1n02x5               g151(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n247));
  oab012aa1n03x5               g152(.a(new_n247), .b(new_n197), .c(new_n205), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n243), .b(new_n227), .c(new_n248), .d(new_n246), .o1(new_n249));
  oa0012aa1n06x5               g154(.a(new_n238), .b(new_n237), .c(new_n232), .o(new_n250));
  inv040aa1n03x5               g155(.a(new_n250), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(new_n249), .b(new_n251), .o1(new_n252));
  nanb02aa1n06x5               g157(.a(new_n252), .b(new_n245), .out0(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n228), .d(new_n243), .o1(new_n255));
  aoi022aa1n02x5               g160(.a(new_n253), .b(new_n254), .c(new_n245), .d(new_n255), .o1(\s[23] ));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[23] ), .b(\a[24] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n258), .b(new_n257), .c(new_n253), .d(new_n254), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n254), .b(new_n252), .c(new_n196), .d(new_n244), .o1(new_n260));
  nona22aa1n02x5               g165(.a(new_n260), .b(new_n258), .c(new_n257), .out0(new_n261));
  nanp02aa1n02x5               g166(.a(new_n259), .b(new_n261), .o1(\s[24] ));
  norb02aa1n15x5               g167(.a(new_n254), .b(new_n258), .out0(new_n263));
  nano22aa1n03x7               g168(.a(new_n230), .b(new_n263), .c(new_n243), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n204), .c(new_n165), .d(new_n187), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n263), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n267));
  aob012aa1n02x5               g172(.a(new_n267), .b(\b[23] ), .c(\a[24] ), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n266), .c(new_n249), .d(new_n251), .o1(new_n269));
  nanb02aa1n02x5               g174(.a(new_n269), .b(new_n265), .out0(new_n270));
  xorc02aa1n12x5               g175(.a(\a[25] ), .b(\b[24] ), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n263), .b(new_n250), .c(new_n228), .d(new_n243), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n271), .b(new_n272), .c(new_n268), .out0(new_n273));
  aoi022aa1n02x5               g178(.a(new_n270), .b(new_n271), .c(new_n265), .d(new_n273), .o1(\s[25] ));
  norp02aa1n02x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[26] ), .b(\b[25] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n275), .c(new_n270), .d(new_n271), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n271), .b(new_n269), .c(new_n196), .d(new_n264), .o1(new_n279));
  nona22aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n275), .out0(new_n280));
  nanp02aa1n02x5               g185(.a(new_n278), .b(new_n280), .o1(\s[26] ));
  nanp02aa1n12x5               g186(.a(new_n276), .b(new_n271), .o1(new_n282));
  nano23aa1n06x5               g187(.a(new_n230), .b(new_n282), .c(new_n263), .d(new_n243), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n204), .c(new_n165), .d(new_n187), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(\b[25] ), .b(\a[26] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n282), .o1(new_n286));
  oai022aa1n02x5               g191(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n287));
  aoi022aa1n09x5               g192(.a(new_n269), .b(new_n286), .c(new_n285), .d(new_n287), .o1(new_n288));
  nand42aa1n04x5               g193(.a(new_n288), .b(new_n284), .o1(new_n289));
  xorc02aa1n12x5               g194(.a(\a[27] ), .b(\b[26] ), .out0(new_n290));
  aoi122aa1n02x5               g195(.a(new_n290), .b(new_n285), .c(new_n287), .d(new_n269), .e(new_n286), .o1(new_n291));
  aoi022aa1n02x5               g196(.a(new_n289), .b(new_n290), .c(new_n291), .d(new_n284), .o1(\s[27] ));
  norp02aa1n02x5               g197(.a(\b[26] ), .b(\a[27] ), .o1(new_n293));
  xnrc02aa1n12x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n293), .c(new_n289), .d(new_n290), .o1(new_n295));
  aobi12aa1n06x5               g200(.a(new_n283), .b(new_n188), .c(new_n195), .out0(new_n296));
  nanp02aa1n02x5               g201(.a(new_n287), .b(new_n285), .o1(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n282), .c(new_n272), .d(new_n268), .o1(new_n298));
  oai012aa1n03x5               g203(.a(new_n290), .b(new_n298), .c(new_n296), .o1(new_n299));
  nona22aa1n03x5               g204(.a(new_n299), .b(new_n294), .c(new_n293), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n295), .b(new_n300), .o1(\s[28] ));
  norb02aa1n03x5               g206(.a(new_n290), .b(new_n294), .out0(new_n302));
  oai012aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n296), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  orn002aa1n02x5               g209(.a(\a[27] ), .b(\b[26] ), .o(new_n305));
  oao003aa1n03x5               g210(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n306));
  aoai13aa1n02x7               g211(.a(new_n306), .b(new_n304), .c(new_n288), .d(new_n284), .o1(new_n307));
  xorc02aa1n12x5               g212(.a(\a[29] ), .b(\b[28] ), .out0(new_n308));
  norb02aa1n02x5               g213(.a(new_n306), .b(new_n308), .out0(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n303), .d(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g216(.a(new_n294), .b(new_n290), .c(new_n308), .out0(new_n312));
  oai012aa1n03x5               g217(.a(new_n312), .b(new_n298), .c(new_n296), .o1(new_n313));
  inv000aa1n02x5               g218(.a(new_n312), .o1(new_n314));
  oaoi03aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .c(new_n306), .o1(new_n315));
  inv000aa1n03x5               g220(.a(new_n315), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n314), .c(new_n288), .d(new_n284), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .out0(new_n318));
  and002aa1n02x5               g223(.a(\b[28] ), .b(\a[29] ), .o(new_n319));
  oabi12aa1n02x5               g224(.a(new_n318), .b(\a[29] ), .c(\b[28] ), .out0(new_n320));
  oab012aa1n02x4               g225(.a(new_n320), .b(new_n306), .c(new_n319), .out0(new_n321));
  aoi022aa1n03x5               g226(.a(new_n317), .b(new_n318), .c(new_n313), .d(new_n321), .o1(\s[30] ));
  nanp03aa1n02x5               g227(.a(new_n302), .b(new_n308), .c(new_n318), .o1(new_n323));
  oabi12aa1n03x5               g228(.a(new_n323), .b(new_n298), .c(new_n296), .out0(new_n324));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  oao003aa1n02x5               g230(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n326));
  norb02aa1n02x5               g231(.a(new_n326), .b(new_n325), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n326), .b(new_n323), .c(new_n288), .d(new_n284), .o1(new_n328));
  aoi022aa1n03x5               g233(.a(new_n328), .b(new_n325), .c(new_n324), .d(new_n327), .o1(\s[31] ));
  xnrb03aa1n02x5               g234(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norb02aa1n02x5               g235(.a(new_n105), .b(new_n104), .out0(new_n331));
  aoi112aa1n02x5               g236(.a(new_n106), .b(new_n331), .c(new_n157), .d(new_n107), .o1(new_n332));
  aoib12aa1n02x5               g237(.a(new_n332), .b(new_n110), .c(new_n104), .out0(\s[4] ));
  nanp02aa1n02x5               g238(.a(new_n158), .b(new_n157), .o1(new_n334));
  xnbna2aa1n03x5               g239(.a(new_n119), .b(new_n334), .c(new_n109), .out0(\s[5] ));
  tech160nm_fioaoi03aa1n03p5x5 g240(.a(\a[5] ), .b(\b[4] ), .c(new_n159), .o1(new_n336));
  xorb03aa1n02x5               g241(.a(new_n336), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g242(.a(new_n161), .b(new_n117), .c(new_n336), .d(new_n116), .o1(new_n338));
  aoi112aa1n02x5               g243(.a(new_n117), .b(new_n161), .c(new_n336), .d(new_n116), .o1(new_n339));
  norb02aa1n02x5               g244(.a(new_n338), .b(new_n339), .out0(\s[7] ));
  xobna2aa1n03x5               g245(.a(new_n160), .b(new_n338), .c(new_n123), .out0(\s[8] ));
  aoi112aa1n02x5               g246(.a(new_n126), .b(new_n127), .c(new_n120), .d(new_n110), .o1(new_n342));
  aoi012aa1n02x5               g247(.a(new_n342), .b(new_n165), .c(new_n127), .o1(\s[9] ));
endmodule


