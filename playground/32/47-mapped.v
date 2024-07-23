// Benchmark "adder" written by ABC on Thu Jul 18 04:47:49 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n309, new_n310,
    new_n311, new_n314, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d32x5               g002(.a(\b[8] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  tech160nm_finor002aa1n03p5x5 g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  tech160nm_finor002aa1n03p5x5 g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  nand42aa1n03x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor042aa1n04x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  oai112aa1n06x5               g011(.a(new_n103), .b(new_n104), .c(new_n105), .d(new_n106), .o1(new_n107));
  aoi022aa1n09x5               g012(.a(new_n107), .b(new_n102), .c(\a[8] ), .d(\b[7] ), .o1(new_n108));
  xnrc02aa1n06x5               g013(.a(\b[2] ), .b(\a[3] ), .out0(new_n109));
  orn002aa1n02x5               g014(.a(\a[2] ), .b(\b[1] ), .o(new_n110));
  nand42aa1n02x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  aob012aa1n06x5               g016(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[3] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[2] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  aoi012aa1n02x5               g020(.a(new_n115), .b(new_n113), .c(new_n114), .o1(new_n116));
  aoai13aa1n04x5               g021(.a(new_n116), .b(new_n109), .c(new_n112), .d(new_n110), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  norb02aa1n02x5               g023(.a(new_n104), .b(new_n101), .out0(new_n119));
  aoi012aa1n02x5               g024(.a(new_n100), .b(\a[4] ), .c(\b[3] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nona23aa1n02x4               g026(.a(new_n103), .b(new_n121), .c(new_n106), .d(new_n105), .out0(new_n122));
  nano32aa1n03x7               g027(.a(new_n122), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n123));
  nanp02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nand22aa1n04x5               g029(.a(new_n99), .b(new_n124), .o1(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n108), .c(new_n123), .d(new_n117), .o1(new_n127));
  nor002aa1d32x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1d28x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nanb02aa1n09x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  aoai13aa1n12x5               g037(.a(new_n129), .b(new_n128), .c(new_n97), .d(new_n98), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  oab012aa1n03x5               g039(.a(new_n134), .b(new_n127), .c(new_n130), .out0(new_n135));
  xnrb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g041(.a(\a[11] ), .b(\b[10] ), .c(new_n135), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand42aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nor022aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nona23aa1n08x5               g047(.a(new_n142), .b(new_n140), .c(new_n139), .d(new_n141), .out0(new_n143));
  nor043aa1n02x5               g048(.a(new_n143), .b(new_n130), .c(new_n125), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n108), .c(new_n123), .d(new_n117), .o1(new_n145));
  aoi012aa1n06x5               g050(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n146));
  oai012aa1d24x5               g051(.a(new_n146), .b(new_n143), .c(new_n133), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nor022aa1n08x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nand42aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n145), .c(new_n148), .out0(\s[13] ));
  nanp02aa1n02x5               g057(.a(new_n145), .b(new_n148), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n149), .b(new_n153), .c(new_n150), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n04x5   g060(.a(\b[14] ), .b(\a[15] ), .out0(new_n156));
  nor002aa1n16x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand02aa1n06x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  oaih12aa1n12x5               g063(.a(new_n158), .b(new_n157), .c(new_n149), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n157), .b(new_n158), .out0(new_n160));
  aoi112aa1n09x5               g065(.a(new_n160), .b(new_n151), .c(new_n145), .d(new_n148), .o1(new_n161));
  inv000aa1n03x5               g066(.a(new_n161), .o1(new_n162));
  xobna2aa1n03x5               g067(.a(new_n156), .b(new_n162), .c(new_n159), .out0(\s[15] ));
  inv000aa1d42x5               g068(.a(\a[15] ), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\b[14] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(new_n165), .b(new_n164), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n159), .o1(new_n167));
  oabi12aa1n06x5               g072(.a(new_n156), .b(new_n161), .c(new_n167), .out0(new_n168));
  xnrc02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .out0(new_n169));
  nand43aa1n02x5               g074(.a(new_n168), .b(new_n166), .c(new_n169), .o1(new_n170));
  tech160nm_fiaoi012aa1n02p5x5 g075(.a(new_n169), .b(new_n168), .c(new_n166), .o1(new_n171));
  norb02aa1n03x4               g076(.a(new_n170), .b(new_n171), .out0(\s[16] ));
  inv000aa1n02x5               g077(.a(new_n108), .o1(new_n173));
  nand42aa1n02x5               g078(.a(new_n123), .b(new_n117), .o1(new_n174));
  nona23aa1n03x5               g079(.a(new_n158), .b(new_n150), .c(new_n149), .d(new_n157), .out0(new_n175));
  nor043aa1n03x5               g080(.a(new_n175), .b(new_n169), .c(new_n156), .o1(new_n176));
  nand02aa1n03x5               g081(.a(new_n176), .b(new_n144), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[16] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[15] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  oai022aa1n02x5               g085(.a(new_n164), .b(new_n165), .c(new_n179), .d(new_n178), .o1(new_n181));
  aoai13aa1n09x5               g086(.a(new_n180), .b(new_n181), .c(new_n159), .d(new_n166), .o1(new_n182));
  aoi012aa1d18x5               g087(.a(new_n182), .b(new_n147), .c(new_n176), .o1(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n177), .c(new_n174), .d(new_n173), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nanp02aa1n02x5               g090(.a(new_n107), .b(new_n102), .o1(new_n186));
  aoi022aa1n06x5               g091(.a(new_n123), .b(new_n117), .c(new_n118), .d(new_n186), .o1(new_n187));
  xorc02aa1n12x5               g092(.a(\a[17] ), .b(\b[16] ), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  oaoi13aa1n04x5               g094(.a(new_n189), .b(new_n183), .c(new_n187), .d(new_n177), .o1(new_n190));
  oab012aa1n02x4               g095(.a(new_n190), .b(\a[17] ), .c(\b[16] ), .out0(new_n191));
  xnrb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  xorc02aa1n02x5               g098(.a(\a[19] ), .b(\b[18] ), .out0(new_n194));
  oai022aa1n04x7               g099(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n195));
  tech160nm_fiaoi012aa1n05x5   g100(.a(new_n195), .b(new_n184), .c(new_n188), .o1(new_n196));
  nano22aa1n03x7               g101(.a(new_n196), .b(new_n193), .c(new_n194), .out0(new_n197));
  oaoi13aa1n02x5               g102(.a(new_n194), .b(new_n193), .c(new_n190), .d(new_n195), .o1(new_n198));
  norp02aa1n02x5               g103(.a(new_n198), .b(new_n197), .o1(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g105(.a(\a[19] ), .o1(new_n201));
  inv000aa1d42x5               g106(.a(\b[18] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  nor042aa1n02x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand42aa1n02x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  nano22aa1n03x7               g111(.a(new_n197), .b(new_n203), .c(new_n206), .out0(new_n207));
  oai112aa1n02x5               g112(.a(new_n194), .b(new_n193), .c(new_n190), .d(new_n195), .o1(new_n208));
  aoi012aa1n02x5               g113(.a(new_n206), .b(new_n208), .c(new_n203), .o1(new_n209));
  norp02aa1n02x5               g114(.a(new_n209), .b(new_n207), .o1(\s[20] ));
  xnrc02aa1n02x5               g115(.a(\b[17] ), .b(\a[18] ), .out0(new_n211));
  nanp02aa1n02x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanb03aa1n06x5               g117(.a(new_n204), .b(new_n205), .c(new_n212), .out0(new_n213));
  nano23aa1n03x7               g118(.a(new_n213), .b(new_n211), .c(new_n188), .d(new_n203), .out0(new_n214));
  nanp03aa1n02x5               g119(.a(new_n195), .b(new_n203), .c(new_n193), .o1(new_n215));
  aoi013aa1n02x4               g120(.a(new_n204), .b(new_n205), .c(new_n201), .d(new_n202), .o1(new_n216));
  oai012aa1n06x5               g121(.a(new_n216), .b(new_n215), .c(new_n213), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[21] ), .b(\b[20] ), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n217), .c(new_n184), .d(new_n214), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(new_n218), .b(new_n217), .c(new_n184), .d(new_n214), .o1(new_n220));
  norb02aa1n02x5               g125(.a(new_n219), .b(new_n220), .out0(\s[21] ));
  inv000aa1d42x5               g126(.a(\a[21] ), .o1(new_n222));
  nanb02aa1n12x5               g127(.a(\b[20] ), .b(new_n222), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n219), .c(new_n223), .out0(\s[22] ));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  xroi22aa1d04x5               g131(.a(new_n222), .b(\b[20] ), .c(new_n226), .d(\b[21] ), .out0(new_n227));
  nanp03aa1n03x5               g132(.a(new_n184), .b(new_n214), .c(new_n227), .o1(new_n228));
  oao003aa1n06x5               g133(.a(\a[22] ), .b(\b[21] ), .c(new_n223), .carry(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoi012aa1n02x5               g135(.a(new_n230), .b(new_n217), .c(new_n227), .o1(new_n231));
  xnrc02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .out0(new_n232));
  xobna2aa1n03x5               g137(.a(new_n232), .b(new_n228), .c(new_n231), .out0(\s[23] ));
  nor042aa1n03x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  tech160nm_fiaoi012aa1n05x5   g140(.a(new_n232), .b(new_n228), .c(new_n231), .o1(new_n236));
  xnrc02aa1n02x5               g141(.a(\b[23] ), .b(\a[24] ), .out0(new_n237));
  nano22aa1n03x7               g142(.a(new_n236), .b(new_n235), .c(new_n237), .out0(new_n238));
  inv000aa1n02x5               g143(.a(new_n231), .o1(new_n239));
  aoi013aa1n02x4               g144(.a(new_n239), .b(new_n184), .c(new_n214), .d(new_n227), .o1(new_n240));
  oaoi13aa1n02x7               g145(.a(new_n237), .b(new_n235), .c(new_n240), .d(new_n232), .o1(new_n241));
  nor002aa1n02x5               g146(.a(new_n241), .b(new_n238), .o1(\s[24] ));
  inv030aa1n02x5               g147(.a(new_n214), .o1(new_n243));
  nano23aa1n03x7               g148(.a(new_n237), .b(new_n232), .c(new_n224), .d(new_n218), .out0(new_n244));
  inv000aa1n02x5               g149(.a(new_n244), .o1(new_n245));
  nona22aa1n06x5               g150(.a(new_n184), .b(new_n243), .c(new_n245), .out0(new_n246));
  norp02aa1n03x5               g151(.a(new_n237), .b(new_n232), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n230), .c(new_n217), .d(new_n227), .o1(new_n248));
  oao003aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .c(new_n235), .carry(new_n249));
  nanp02aa1n06x5               g154(.a(new_n248), .b(new_n249), .o1(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  tech160nm_fixnrc02aa1n05x5   g156(.a(\b[24] ), .b(\a[25] ), .out0(new_n252));
  xobna2aa1n03x5               g157(.a(new_n252), .b(new_n246), .c(new_n251), .out0(\s[25] ));
  nor042aa1n03x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  tech160nm_fiaoi012aa1n05x5   g160(.a(new_n252), .b(new_n246), .c(new_n251), .o1(new_n256));
  xnrc02aa1n06x5               g161(.a(\b[25] ), .b(\a[26] ), .out0(new_n257));
  nano22aa1n03x7               g162(.a(new_n256), .b(new_n255), .c(new_n257), .out0(new_n258));
  aoi013aa1n03x5               g163(.a(new_n250), .b(new_n184), .c(new_n214), .d(new_n244), .o1(new_n259));
  oaoi13aa1n03x5               g164(.a(new_n257), .b(new_n255), .c(new_n259), .d(new_n252), .o1(new_n260));
  nor002aa1n02x5               g165(.a(new_n260), .b(new_n258), .o1(\s[26] ));
  nor042aa1n02x5               g166(.a(new_n257), .b(new_n252), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n262), .o1(new_n263));
  nona32aa1n09x5               g168(.a(new_n184), .b(new_n263), .c(new_n245), .d(new_n243), .out0(new_n264));
  oao003aa1n02x5               g169(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n265));
  aobi12aa1n12x5               g170(.a(new_n265), .b(new_n250), .c(new_n262), .out0(new_n266));
  nor042aa1n03x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  and002aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n267), .o1(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n266), .c(new_n264), .out0(\s[27] ));
  inv000aa1d42x5               g175(.a(new_n267), .o1(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[27] ), .b(\a[28] ), .out0(new_n272));
  oaoi13aa1n04x5               g177(.a(new_n243), .b(new_n183), .c(new_n187), .d(new_n177), .o1(new_n273));
  and003aa1n02x5               g178(.a(new_n227), .b(new_n262), .c(new_n247), .o(new_n274));
  aoai13aa1n02x7               g179(.a(new_n265), .b(new_n263), .c(new_n248), .d(new_n249), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n268), .o1(new_n276));
  aoai13aa1n02x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .d(new_n274), .o1(new_n277));
  aoi012aa1n02x5               g182(.a(new_n272), .b(new_n277), .c(new_n271), .o1(new_n278));
  aoi022aa1n03x5               g183(.a(new_n266), .b(new_n264), .c(\a[27] ), .d(\b[26] ), .o1(new_n279));
  nano22aa1n03x5               g184(.a(new_n279), .b(new_n271), .c(new_n272), .out0(new_n280));
  norp02aa1n03x5               g185(.a(new_n278), .b(new_n280), .o1(\s[28] ));
  nano22aa1n02x4               g186(.a(new_n272), .b(new_n276), .c(new_n271), .out0(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n275), .c(new_n273), .d(new_n274), .o1(new_n283));
  oao003aa1n02x5               g188(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[28] ), .b(\a[29] ), .out0(new_n285));
  aoi012aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n284), .o1(new_n286));
  aobi12aa1n06x5               g191(.a(new_n282), .b(new_n266), .c(new_n264), .out0(new_n287));
  nano22aa1n03x7               g192(.a(new_n287), .b(new_n284), .c(new_n285), .out0(new_n288));
  norp02aa1n03x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n269), .b(new_n285), .c(new_n272), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n275), .c(new_n273), .d(new_n274), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n06x5               g200(.a(new_n291), .b(new_n266), .c(new_n264), .out0(new_n296));
  nano22aa1n03x7               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  norb03aa1n02x5               g203(.a(new_n282), .b(new_n294), .c(new_n285), .out0(new_n299));
  aobi12aa1n06x5               g204(.a(new_n299), .b(new_n266), .c(new_n264), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano22aa1n03x7               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n299), .b(new_n275), .c(new_n273), .d(new_n274), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xobna2aa1n03x5               g211(.a(new_n109), .b(new_n112), .c(new_n110), .out0(\s[3] ));
  nanp02aa1n02x5               g212(.a(\b[3] ), .b(\a[4] ), .o1(new_n308));
  nanp02aa1n02x5               g213(.a(new_n117), .b(new_n308), .o1(new_n309));
  aboi22aa1n03x5               g214(.a(new_n115), .b(new_n308), .c(new_n113), .d(new_n114), .out0(new_n310));
  aoai13aa1n02x5               g215(.a(new_n310), .b(new_n109), .c(new_n112), .d(new_n110), .o1(new_n311));
  oa0012aa1n02x5               g216(.a(new_n311), .b(new_n309), .c(new_n115), .o(\s[4] ));
  xnrb03aa1n02x5               g217(.a(new_n309), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g218(.a(new_n121), .b(new_n106), .c(new_n117), .d(new_n308), .o1(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g220(.a(\a[6] ), .b(\b[5] ), .c(new_n314), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g222(.a(new_n101), .b(new_n316), .c(new_n104), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g224(.a(new_n187), .b(new_n124), .c(new_n99), .out0(\s[9] ));
endmodule


