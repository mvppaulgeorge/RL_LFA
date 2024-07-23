// Benchmark "adder" written by ABC on Thu Jul 18 10:46:41 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n274,
    new_n275, new_n276, new_n277, new_n278, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n319, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n331, new_n332, new_n333, new_n335, new_n337, new_n339, new_n340,
    new_n341, new_n342, new_n343, new_n345, new_n346, new_n347, new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[9] ), .o1(new_n97));
  inv040aa1d28x5               g002(.a(\b[8] ), .o1(new_n98));
  nand02aa1d16x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  and002aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o(new_n100));
  oaoi03aa1n12x5               g005(.a(\a[2] ), .b(\b[1] ), .c(new_n100), .o1(new_n101));
  xorc02aa1n12x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  nor042aa1d18x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n03x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanp03aa1d12x5               g010(.a(new_n101), .b(new_n102), .c(new_n105), .o1(new_n106));
  inv040aa1n08x5               g011(.a(new_n103), .o1(new_n107));
  oaoi03aa1n12x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .o1(new_n108));
  inv000aa1d42x5               g013(.a(new_n108), .o1(new_n109));
  oai022aa1n09x5               g014(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .out0(new_n111));
  inv020aa1n20x5               g016(.a(\a[7] ), .o1(new_n112));
  nanb02aa1n12x5               g017(.a(\b[6] ), .b(new_n112), .out0(new_n113));
  and002aa1n12x5               g018(.a(\b[6] ), .b(\a[7] ), .o(new_n114));
  aoi022aa1d24x5               g019(.a(\b[5] ), .b(\a[6] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n115));
  nano22aa1n03x5               g020(.a(new_n114), .b(new_n115), .c(new_n113), .out0(new_n116));
  nona22aa1n02x4               g021(.a(new_n116), .b(new_n111), .c(new_n110), .out0(new_n117));
  xorc02aa1n12x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  nor002aa1d32x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  aoi112aa1n06x5               g024(.a(new_n114), .b(new_n119), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  oaoi03aa1n03x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n113), .o1(new_n121));
  aoi013aa1n06x4               g026(.a(new_n121), .b(new_n120), .c(new_n118), .d(new_n110), .o1(new_n122));
  aoai13aa1n12x5               g027(.a(new_n122), .b(new_n117), .c(new_n106), .d(new_n109), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(new_n123), .b(new_n124), .o1(new_n125));
  nor022aa1n16x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1n06x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nanb02aa1d24x5               g032(.a(new_n126), .b(new_n127), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n125), .c(new_n99), .out0(\s[10] ));
  tech160nm_fiao0012aa1n02p5x5 g035(.a(new_n117), .b(new_n106), .c(new_n109), .o(new_n131));
  nanb02aa1n09x5               g036(.a(new_n128), .b(new_n124), .out0(new_n132));
  oaoi03aa1n12x5               g037(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n131), .d(new_n122), .o1(new_n135));
  nor042aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1n04x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoi113aa1n02x5               g043(.a(new_n133), .b(new_n138), .c(new_n123), .d(new_n124), .e(new_n129), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n139), .b(new_n135), .c(new_n138), .o1(\s[11] ));
  inv000aa1d42x5               g045(.a(\a[12] ), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n136), .b(new_n135), .c(new_n137), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(new_n141), .out0(\s[12] ));
  nor042aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1d04x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nano23aa1d12x5               g050(.a(new_n136), .b(new_n144), .c(new_n145), .d(new_n137), .out0(new_n146));
  norb02aa1n02x5               g051(.a(new_n146), .b(new_n132), .out0(new_n147));
  oa0012aa1n03x5               g052(.a(new_n145), .b(new_n144), .c(new_n136), .o(new_n148));
  tech160nm_fiao0012aa1n02p5x5 g053(.a(new_n148), .b(new_n146), .c(new_n133), .o(new_n149));
  nor042aa1n06x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n149), .c(new_n123), .d(new_n147), .o1(new_n154));
  aoi112aa1n02x5               g059(.a(new_n148), .b(new_n153), .c(new_n146), .d(new_n133), .o1(new_n155));
  aobi12aa1n02x5               g060(.a(new_n155), .b(new_n123), .c(new_n147), .out0(new_n156));
  norb02aa1n02x5               g061(.a(new_n154), .b(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(new_n150), .o1(new_n158));
  nor042aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n154), .c(new_n158), .out0(\s[14] ));
  nano23aa1d12x5               g067(.a(new_n150), .b(new_n159), .c(new_n160), .d(new_n151), .out0(new_n163));
  nano22aa1n02x4               g068(.a(new_n132), .b(new_n146), .c(new_n163), .out0(new_n164));
  aoai13aa1n06x5               g069(.a(new_n163), .b(new_n148), .c(new_n146), .d(new_n133), .o1(new_n165));
  tech160nm_fioai012aa1n05x5   g070(.a(new_n160), .b(new_n159), .c(new_n150), .o1(new_n166));
  nand02aa1n03x5               g071(.a(new_n165), .b(new_n166), .o1(new_n167));
  xorc02aa1n12x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n167), .c(new_n123), .d(new_n164), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n168), .o1(new_n170));
  nanp03aa1n02x5               g075(.a(new_n165), .b(new_n166), .c(new_n170), .o1(new_n171));
  aoi013aa1n02x4               g076(.a(new_n171), .b(new_n123), .c(new_n147), .d(new_n163), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n169), .b(new_n172), .out0(\s[15] ));
  orn002aa1n02x5               g078(.a(\a[15] ), .b(\b[14] ), .o(new_n174));
  tech160nm_fixnrc02aa1n04x5   g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  xobna2aa1n03x5               g080(.a(new_n175), .b(new_n169), .c(new_n174), .out0(\s[16] ));
  nanb02aa1d24x5               g081(.a(new_n175), .b(new_n168), .out0(new_n177));
  nano23aa1d15x5               g082(.a(new_n177), .b(new_n132), .c(new_n146), .d(new_n163), .out0(new_n178));
  nanp02aa1n06x5               g083(.a(new_n123), .b(new_n178), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n177), .o1(new_n180));
  nand02aa1d04x5               g085(.a(new_n167), .b(new_n180), .o1(new_n181));
  oaoi03aa1n09x5               g086(.a(\a[16] ), .b(\b[15] ), .c(new_n174), .o1(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  nand23aa1n06x5               g088(.a(new_n179), .b(new_n181), .c(new_n183), .o1(new_n184));
  tech160nm_fixorc02aa1n02p5x5 g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  aoi112aa1n02x5               g090(.a(new_n185), .b(new_n182), .c(new_n167), .d(new_n180), .o1(new_n186));
  aoi022aa1n02x5               g091(.a(new_n184), .b(new_n185), .c(new_n179), .d(new_n186), .o1(\s[17] ));
  inv040aa1d32x5               g092(.a(\a[17] ), .o1(new_n188));
  inv040aa1n16x5               g093(.a(\b[16] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  aoai13aa1n04x5               g095(.a(new_n183), .b(new_n177), .c(new_n165), .d(new_n166), .o1(new_n191));
  aoai13aa1n06x5               g096(.a(new_n185), .b(new_n191), .c(new_n123), .d(new_n178), .o1(new_n192));
  nor002aa1n06x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nanp02aa1n04x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  xobna2aa1n03x5               g100(.a(new_n195), .b(new_n192), .c(new_n190), .out0(\s[18] ));
  tech160nm_fiaoi012aa1n03p5x5 g101(.a(new_n177), .b(new_n165), .c(new_n166), .o1(new_n197));
  aoi112aa1n09x5               g102(.a(new_n197), .b(new_n182), .c(new_n123), .d(new_n178), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\a[18] ), .o1(new_n199));
  xroi22aa1d06x4               g104(.a(new_n188), .b(\b[16] ), .c(new_n199), .d(\b[17] ), .out0(new_n200));
  inv030aa1n02x5               g105(.a(new_n200), .o1(new_n201));
  aoai13aa1n12x5               g106(.a(new_n194), .b(new_n193), .c(new_n188), .d(new_n189), .o1(new_n202));
  tech160nm_fioai012aa1n05x5   g107(.a(new_n202), .b(new_n198), .c(new_n201), .o1(new_n203));
  nor002aa1d32x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand42aa1n04x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  oaoi03aa1n03x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n190), .o1(new_n207));
  aoi112aa1n03x4               g112(.a(new_n206), .b(new_n207), .c(new_n184), .d(new_n200), .o1(new_n208));
  tech160nm_fiaoi012aa1n02p5x5 g113(.a(new_n208), .b(new_n203), .c(new_n206), .o1(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nanp02aa1n09x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  aoai13aa1n02x7               g118(.a(new_n213), .b(new_n204), .c(new_n203), .d(new_n206), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n206), .b(new_n207), .c(new_n184), .d(new_n200), .o1(new_n215));
  nona22aa1n03x5               g120(.a(new_n215), .b(new_n213), .c(new_n204), .out0(new_n216));
  nanp02aa1n03x5               g121(.a(new_n214), .b(new_n216), .o1(\s[20] ));
  nona23aa1n09x5               g122(.a(new_n212), .b(new_n205), .c(new_n204), .d(new_n211), .out0(new_n218));
  norb03aa1n03x5               g123(.a(new_n185), .b(new_n218), .c(new_n195), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n191), .c(new_n123), .d(new_n178), .o1(new_n220));
  inv020aa1n02x5               g125(.a(new_n219), .o1(new_n221));
  oa0012aa1n03x5               g126(.a(new_n212), .b(new_n211), .c(new_n204), .o(new_n222));
  inv040aa1n03x5               g127(.a(new_n222), .o1(new_n223));
  oai012aa1n18x5               g128(.a(new_n223), .b(new_n218), .c(new_n202), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  tech160nm_fioai012aa1n04x5   g130(.a(new_n225), .b(new_n198), .c(new_n221), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[20] ), .b(\a[21] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  nano23aa1n03x7               g133(.a(new_n204), .b(new_n211), .c(new_n212), .d(new_n205), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n222), .b(new_n228), .c(new_n229), .d(new_n207), .o1(new_n230));
  aoi022aa1n02x5               g135(.a(new_n226), .b(new_n228), .c(new_n220), .d(new_n230), .o1(\s[21] ));
  nor042aa1d18x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[21] ), .b(\a[22] ), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n232), .c(new_n226), .d(new_n228), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n228), .b(new_n224), .c(new_n184), .d(new_n219), .o1(new_n235));
  nona22aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n232), .out0(new_n236));
  nanp02aa1n03x5               g141(.a(new_n234), .b(new_n236), .o1(\s[22] ));
  nor042aa1n06x5               g142(.a(new_n233), .b(new_n227), .o1(new_n238));
  nano22aa1n02x4               g143(.a(new_n201), .b(new_n238), .c(new_n229), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n191), .c(new_n123), .d(new_n178), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n239), .o1(new_n241));
  inv040aa1d32x5               g146(.a(\a[22] ), .o1(new_n242));
  inv040aa1d32x5               g147(.a(\b[21] ), .o1(new_n243));
  oao003aa1n02x5               g148(.a(new_n242), .b(new_n243), .c(new_n232), .carry(new_n244));
  aoi012aa1d24x5               g149(.a(new_n244), .b(new_n224), .c(new_n238), .o1(new_n245));
  tech160nm_fioai012aa1n04x5   g150(.a(new_n245), .b(new_n198), .c(new_n241), .o1(new_n246));
  xorc02aa1n12x5               g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  aoi112aa1n02x5               g152(.a(new_n247), .b(new_n244), .c(new_n224), .d(new_n238), .o1(new_n248));
  aoi022aa1n02x5               g153(.a(new_n246), .b(new_n247), .c(new_n240), .d(new_n248), .o1(\s[23] ));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norp02aa1n04x5               g155(.a(\b[23] ), .b(\a[24] ), .o1(new_n251));
  nanp02aa1n04x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  nanb02aa1n03x5               g157(.a(new_n251), .b(new_n252), .out0(new_n253));
  aoai13aa1n03x5               g158(.a(new_n253), .b(new_n250), .c(new_n246), .d(new_n247), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n245), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n247), .b(new_n255), .c(new_n184), .d(new_n239), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n256), .b(new_n253), .c(new_n250), .out0(new_n257));
  nanp02aa1n03x5               g162(.a(new_n254), .b(new_n257), .o1(\s[24] ));
  norb02aa1n02x5               g163(.a(new_n247), .b(new_n253), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  nano32aa1n03x7               g165(.a(new_n260), .b(new_n200), .c(new_n238), .d(new_n229), .out0(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n191), .c(new_n123), .d(new_n178), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n261), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n238), .b(new_n222), .c(new_n229), .d(new_n207), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n244), .o1(new_n265));
  oai012aa1n02x5               g170(.a(new_n252), .b(new_n251), .c(new_n250), .o1(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n260), .c(new_n264), .d(new_n265), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fioai012aa1n04x5   g173(.a(new_n268), .b(new_n198), .c(new_n263), .o1(new_n269));
  xorc02aa1n12x5               g174(.a(\a[25] ), .b(\b[24] ), .out0(new_n270));
  aoai13aa1n02x5               g175(.a(new_n259), .b(new_n244), .c(new_n224), .d(new_n238), .o1(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n271), .c(new_n266), .out0(new_n272));
  aoi022aa1n02x5               g177(.a(new_n269), .b(new_n270), .c(new_n262), .d(new_n272), .o1(\s[25] ));
  nor002aa1n02x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  tech160nm_fixnrc02aa1n04x5   g179(.a(\b[25] ), .b(\a[26] ), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n269), .d(new_n270), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n270), .b(new_n267), .c(new_n184), .d(new_n261), .o1(new_n277));
  nona22aa1n02x4               g182(.a(new_n277), .b(new_n275), .c(new_n274), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n276), .b(new_n278), .o1(\s[26] ));
  norb02aa1n06x5               g184(.a(new_n270), .b(new_n275), .out0(new_n280));
  nano23aa1n03x7               g185(.a(new_n221), .b(new_n260), .c(new_n280), .d(new_n238), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n191), .c(new_n123), .d(new_n178), .o1(new_n282));
  inv000aa1d42x5               g187(.a(\a[26] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(\b[25] ), .o1(new_n284));
  oao003aa1n02x5               g189(.a(new_n283), .b(new_n284), .c(new_n274), .carry(new_n285));
  aoi012aa1n06x5               g190(.a(new_n285), .b(new_n267), .c(new_n280), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[27] ), .b(\b[26] ), .out0(new_n287));
  xnbna2aa1n03x5               g192(.a(new_n287), .b(new_n282), .c(new_n286), .out0(\s[27] ));
  inv000aa1n02x5               g193(.a(new_n281), .o1(new_n289));
  tech160nm_fioai012aa1n05x5   g194(.a(new_n286), .b(new_n198), .c(new_n289), .o1(new_n290));
  norp02aa1n02x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  nor002aa1n02x5               g196(.a(\b[27] ), .b(\a[28] ), .o1(new_n292));
  nand02aa1n06x5               g197(.a(\b[27] ), .b(\a[28] ), .o1(new_n293));
  nanb02aa1n02x5               g198(.a(new_n292), .b(new_n293), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n291), .c(new_n290), .d(new_n287), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n280), .o1(new_n296));
  inv000aa1n02x5               g201(.a(new_n285), .o1(new_n297));
  aoai13aa1n02x7               g202(.a(new_n297), .b(new_n296), .c(new_n271), .d(new_n266), .o1(new_n298));
  aoai13aa1n02x7               g203(.a(new_n287), .b(new_n298), .c(new_n184), .d(new_n281), .o1(new_n299));
  nona22aa1n02x4               g204(.a(new_n299), .b(new_n294), .c(new_n291), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n295), .b(new_n300), .o1(\s[28] ));
  norb02aa1n02x7               g206(.a(new_n287), .b(new_n294), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n298), .c(new_n184), .d(new_n281), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n302), .o1(new_n304));
  oai012aa1n02x5               g209(.a(new_n293), .b(new_n292), .c(new_n291), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n304), .c(new_n282), .d(new_n286), .o1(new_n306));
  nor002aa1n02x5               g211(.a(\b[28] ), .b(\a[29] ), .o1(new_n307));
  nand42aa1n08x5               g212(.a(\b[28] ), .b(\a[29] ), .o1(new_n308));
  norb02aa1n02x7               g213(.a(new_n308), .b(new_n307), .out0(new_n309));
  oai022aa1n02x5               g214(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n310));
  aboi22aa1n03x5               g215(.a(new_n307), .b(new_n308), .c(new_n310), .d(new_n293), .out0(new_n311));
  aoi022aa1n03x5               g216(.a(new_n306), .b(new_n309), .c(new_n303), .d(new_n311), .o1(\s[29] ));
  xnrb03aa1n02x5               g217(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g218(.a(new_n294), .b(new_n287), .c(new_n309), .out0(new_n314));
  nanb02aa1n03x5               g219(.a(new_n314), .b(new_n290), .out0(new_n315));
  aoi013aa1n02x4               g220(.a(new_n307), .b(new_n310), .c(new_n293), .d(new_n308), .o1(new_n316));
  aoai13aa1n02x5               g221(.a(new_n316), .b(new_n314), .c(new_n282), .d(new_n286), .o1(new_n317));
  xorc02aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .out0(new_n318));
  aoi113aa1n02x5               g223(.a(new_n318), .b(new_n307), .c(new_n310), .d(new_n308), .e(new_n293), .o1(new_n319));
  aoi022aa1n03x5               g224(.a(new_n317), .b(new_n318), .c(new_n315), .d(new_n319), .o1(\s[30] ));
  inv000aa1n02x5               g225(.a(new_n309), .o1(new_n321));
  nona23aa1n02x4               g226(.a(new_n287), .b(new_n318), .c(new_n321), .d(new_n294), .out0(new_n322));
  nanb02aa1n03x5               g227(.a(new_n322), .b(new_n290), .out0(new_n323));
  oao003aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .c(new_n316), .carry(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n322), .c(new_n282), .d(new_n286), .o1(new_n325));
  xorc02aa1n02x5               g230(.a(\a[31] ), .b(\b[30] ), .out0(new_n326));
  and002aa1n02x5               g231(.a(\b[29] ), .b(\a[30] ), .o(new_n327));
  oabi12aa1n02x5               g232(.a(new_n326), .b(\a[30] ), .c(\b[29] ), .out0(new_n328));
  oab012aa1n02x4               g233(.a(new_n328), .b(new_n316), .c(new_n327), .out0(new_n329));
  aoi022aa1n03x5               g234(.a(new_n325), .b(new_n326), .c(new_n323), .d(new_n329), .o1(\s[31] ));
  orn002aa1n02x5               g235(.a(\a[2] ), .b(\b[1] ), .o(new_n331));
  nanp02aa1n02x5               g236(.a(\b[1] ), .b(\a[2] ), .o1(new_n332));
  nanb03aa1n02x5               g237(.a(new_n100), .b(new_n331), .c(new_n332), .out0(new_n333));
  xnbna2aa1n03x5               g238(.a(new_n105), .b(new_n333), .c(new_n331), .out0(\s[3] ));
  nanp02aa1n02x5               g239(.a(new_n101), .b(new_n105), .o1(new_n335));
  xnbna2aa1n03x5               g240(.a(new_n102), .b(new_n335), .c(new_n107), .out0(\s[4] ));
  xorc02aa1n02x5               g241(.a(\a[5] ), .b(\b[4] ), .out0(new_n337));
  xnbna2aa1n03x5               g242(.a(new_n337), .b(new_n106), .c(new_n109), .out0(\s[5] ));
  inv000aa1d42x5               g243(.a(\a[5] ), .o1(new_n339));
  inv000aa1d42x5               g244(.a(\b[4] ), .o1(new_n340));
  nanp02aa1n02x5               g245(.a(new_n340), .b(new_n339), .o1(new_n341));
  aob012aa1n02x5               g246(.a(new_n337), .b(new_n106), .c(new_n109), .out0(new_n342));
  xorc02aa1n02x5               g247(.a(\a[6] ), .b(\b[5] ), .out0(new_n343));
  xnbna2aa1n03x5               g248(.a(new_n343), .b(new_n342), .c(new_n341), .out0(\s[6] ));
  aobi12aa1n02x5               g249(.a(new_n343), .b(new_n342), .c(new_n341), .out0(new_n345));
  oaib12aa1n02x5               g250(.a(new_n120), .b(new_n110), .c(new_n342), .out0(new_n346));
  oai022aa1n02x5               g251(.a(new_n114), .b(new_n119), .c(\b[5] ), .d(\a[6] ), .o1(new_n347));
  oa0012aa1n02x5               g252(.a(new_n346), .b(new_n345), .c(new_n347), .o(\s[7] ));
  xnbna2aa1n03x5               g253(.a(new_n118), .b(new_n346), .c(new_n113), .out0(\s[8] ));
  aoi113aa1n02x5               g254(.a(new_n124), .b(new_n121), .c(new_n120), .d(new_n118), .e(new_n110), .o1(new_n350));
  aoi022aa1n02x5               g255(.a(new_n131), .b(new_n350), .c(new_n123), .d(new_n124), .o1(\s[9] ));
endmodule


