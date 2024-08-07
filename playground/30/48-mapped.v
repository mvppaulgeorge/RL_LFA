// Benchmark "adder" written by ABC on Thu Jul 18 03:46:57 2024

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
    new_n118, new_n119, new_n120, new_n122, new_n123, new_n124, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n145, new_n146, new_n147, new_n149, new_n150,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n157, new_n158,
    new_n159, new_n161, new_n162, new_n163, new_n164, new_n165, new_n166,
    new_n168, new_n169, new_n170, new_n171, new_n173, new_n174, new_n175,
    new_n176, new_n177, new_n178, new_n179, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n188, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n209, new_n210, new_n211, new_n212, new_n213, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n244, new_n245, new_n246,
    new_n247, new_n248, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n268, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n295, new_n296,
    new_n297, new_n300, new_n302, new_n304;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[2] ), .b(\a[3] ), .out0(new_n97));
  nand42aa1n10x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor022aa1n16x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  oai012aa1d24x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  oa0022aa1n12x5               g006(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n102));
  oai012aa1n18x5               g007(.a(new_n102), .b(new_n97), .c(new_n101), .o1(new_n103));
  nand02aa1n04x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nor042aa1n12x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nand02aa1d24x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nand42aa1n04x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanb03aa1n03x5               g012(.a(new_n105), .b(new_n107), .c(new_n106), .out0(new_n108));
  nor042aa1d18x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nand02aa1n03x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanb02aa1n06x5               g015(.a(new_n109), .b(new_n110), .out0(new_n111));
  nor022aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n03x5               g019(.a(new_n114), .b(new_n104), .c(new_n112), .d(new_n113), .out0(new_n115));
  nor003aa1n03x5               g020(.a(new_n115), .b(new_n111), .c(new_n108), .o1(new_n116));
  aoai13aa1n02x7               g021(.a(new_n114), .b(new_n105), .c(new_n109), .d(new_n106), .o1(new_n117));
  nona22aa1n03x5               g022(.a(new_n117), .b(new_n113), .c(new_n112), .out0(new_n118));
  aoi022aa1n12x5               g023(.a(new_n116), .b(new_n103), .c(new_n118), .d(new_n104), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[9] ), .b(\b[8] ), .c(new_n119), .o1(new_n120));
  xorb03aa1n02x5               g025(.a(new_n120), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  tech160nm_fixorc02aa1n02p5x5 g026(.a(\a[10] ), .b(\b[9] ), .out0(new_n122));
  oai022aa1d18x5               g027(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n123));
  aob012aa1d18x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  nor002aa1d32x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nand22aa1n03x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n120), .d(new_n122), .o1(new_n129));
  aoi112aa1n02x5               g034(.a(new_n128), .b(new_n125), .c(new_n120), .d(new_n122), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n129), .b(new_n130), .out0(\s[11] ));
  inv040aa1n04x5               g036(.a(new_n126), .o1(new_n132));
  nor042aa1n09x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  nand42aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n135), .b(new_n129), .c(new_n132), .out0(\s[12] ));
  xorc02aa1n03x5               g041(.a(\a[9] ), .b(\b[8] ), .out0(new_n137));
  nano23aa1n03x5               g042(.a(new_n126), .b(new_n133), .c(new_n134), .d(new_n127), .out0(new_n138));
  nanp03aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n122), .o1(new_n139));
  nona23aa1n09x5               g044(.a(new_n134), .b(new_n127), .c(new_n126), .d(new_n133), .out0(new_n140));
  oaoi03aa1n09x5               g045(.a(\a[12] ), .b(\b[11] ), .c(new_n132), .o1(new_n141));
  oabi12aa1n06x5               g046(.a(new_n141), .b(new_n140), .c(new_n124), .out0(new_n142));
  oabi12aa1n06x5               g047(.a(new_n142), .b(new_n119), .c(new_n139), .out0(new_n143));
  xorb03aa1n02x5               g048(.a(new_n143), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d32x5               g049(.a(\a[13] ), .o1(new_n145));
  inv040aa1d28x5               g050(.a(\b[12] ), .o1(new_n146));
  oaoi03aa1n02x5               g051(.a(new_n145), .b(new_n146), .c(new_n143), .o1(new_n147));
  xnrb03aa1n02x5               g052(.a(new_n147), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv020aa1n04x5               g053(.a(\a[14] ), .o1(new_n149));
  xroi22aa1d06x4               g054(.a(new_n145), .b(\b[12] ), .c(new_n149), .d(\b[13] ), .out0(new_n150));
  tech160nm_finand02aa1n03p5x5 g055(.a(new_n146), .b(new_n145), .o1(new_n151));
  oaoi03aa1n12x5               g056(.a(\a[14] ), .b(\b[13] ), .c(new_n151), .o1(new_n152));
  xorc02aa1n12x5               g057(.a(\a[15] ), .b(\b[14] ), .out0(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n152), .c(new_n143), .d(new_n150), .o1(new_n154));
  aoi112aa1n02x5               g059(.a(new_n153), .b(new_n152), .c(new_n143), .d(new_n150), .o1(new_n155));
  norb02aa1n02x5               g060(.a(new_n154), .b(new_n155), .out0(\s[15] ));
  norp02aa1n02x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n157), .o1(new_n158));
  xorc02aa1n12x5               g063(.a(\a[16] ), .b(\b[15] ), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n154), .c(new_n158), .out0(\s[16] ));
  nanp02aa1n02x5               g065(.a(new_n122), .b(new_n137), .o1(new_n161));
  and002aa1n12x5               g066(.a(new_n159), .b(new_n153), .o(new_n162));
  nona23aa1n09x5               g067(.a(new_n162), .b(new_n150), .c(new_n161), .d(new_n140), .out0(new_n163));
  aoai13aa1n02x7               g068(.a(new_n162), .b(new_n152), .c(new_n142), .d(new_n150), .o1(new_n164));
  oao003aa1n02x5               g069(.a(\a[16] ), .b(\b[15] ), .c(new_n158), .carry(new_n165));
  oai112aa1n06x5               g070(.a(new_n164), .b(new_n165), .c(new_n119), .d(new_n163), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g072(.a(\a[18] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(\a[17] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(\b[16] ), .o1(new_n170));
  oaoi03aa1n03x5               g075(.a(new_n169), .b(new_n170), .c(new_n166), .o1(new_n171));
  xorb03aa1n03x5               g076(.a(new_n171), .b(\b[17] ), .c(new_n168), .out0(\s[18] ));
  nor042aa1n02x5               g077(.a(new_n119), .b(new_n163), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n152), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n162), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n150), .b(new_n141), .c(new_n125), .d(new_n138), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n165), .b(new_n175), .c(new_n176), .d(new_n174), .o1(new_n177));
  xroi22aa1d06x4               g082(.a(new_n169), .b(\b[16] ), .c(new_n168), .d(\b[17] ), .out0(new_n178));
  oai012aa1n02x5               g083(.a(new_n178), .b(new_n177), .c(new_n173), .o1(new_n179));
  oai022aa1n02x5               g084(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n180));
  oaib12aa1n02x5               g085(.a(new_n180), .b(new_n168), .c(\b[17] ), .out0(new_n181));
  nor002aa1n20x5               g086(.a(\b[18] ), .b(\a[19] ), .o1(new_n182));
  nand02aa1n03x5               g087(.a(\b[18] ), .b(\a[19] ), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n179), .c(new_n181), .out0(\s[19] ));
  xnrc02aa1n02x5               g091(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g092(.a(new_n182), .o1(new_n188));
  nanp02aa1n02x5               g093(.a(new_n170), .b(new_n169), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n190));
  aoai13aa1n03x5               g095(.a(new_n185), .b(new_n190), .c(new_n166), .d(new_n178), .o1(new_n191));
  nor042aa1n06x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  nand22aa1n06x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(new_n194));
  tech160nm_fiaoi012aa1n02p5x5 g099(.a(new_n194), .b(new_n191), .c(new_n188), .o1(new_n195));
  nanp03aa1n03x5               g100(.a(new_n191), .b(new_n188), .c(new_n194), .o1(new_n196));
  norb02aa1n03x4               g101(.a(new_n196), .b(new_n195), .out0(\s[20] ));
  nano23aa1n06x5               g102(.a(new_n182), .b(new_n192), .c(new_n193), .d(new_n183), .out0(new_n198));
  nand02aa1n04x5               g103(.a(new_n178), .b(new_n198), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nona23aa1n09x5               g105(.a(new_n193), .b(new_n183), .c(new_n182), .d(new_n192), .out0(new_n201));
  aoi012aa1d18x5               g106(.a(new_n192), .b(new_n182), .c(new_n193), .o1(new_n202));
  oaih12aa1n04x5               g107(.a(new_n202), .b(new_n201), .c(new_n181), .o1(new_n203));
  tech160nm_fiaoi012aa1n05x5   g108(.a(new_n203), .b(new_n166), .c(new_n200), .o1(new_n204));
  nor042aa1n03x5               g109(.a(\b[20] ), .b(\a[21] ), .o1(new_n205));
  inv000aa1n06x5               g110(.a(new_n205), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[20] ), .b(\a[21] ), .o1(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n204), .b(new_n207), .c(new_n206), .out0(\s[21] ));
  norb02aa1n02x5               g113(.a(new_n207), .b(new_n205), .out0(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n203), .c(new_n166), .d(new_n200), .o1(new_n210));
  xnrc02aa1n02x5               g115(.a(\b[21] ), .b(\a[22] ), .out0(new_n211));
  aoi012aa1n02x7               g116(.a(new_n211), .b(new_n210), .c(new_n206), .o1(new_n212));
  nanp03aa1n03x5               g117(.a(new_n210), .b(new_n206), .c(new_n211), .o1(new_n213));
  norb02aa1n03x4               g118(.a(new_n213), .b(new_n212), .out0(\s[22] ));
  nano22aa1n03x7               g119(.a(new_n211), .b(new_n206), .c(new_n207), .out0(new_n215));
  and003aa1n02x5               g120(.a(new_n178), .b(new_n215), .c(new_n198), .o(new_n216));
  oai012aa1n02x5               g121(.a(new_n216), .b(new_n177), .c(new_n173), .o1(new_n217));
  oao003aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .c(new_n206), .carry(new_n218));
  inv040aa1n02x5               g123(.a(new_n218), .o1(new_n219));
  aoi012aa1n02x5               g124(.a(new_n219), .b(new_n203), .c(new_n215), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[22] ), .b(\a[23] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n220), .out0(\s[23] ));
  nor042aa1n03x5               g128(.a(\b[22] ), .b(\a[23] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n220), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n222), .b(new_n226), .c(new_n166), .d(new_n216), .o1(new_n227));
  xnrc02aa1n02x5               g132(.a(\b[23] ), .b(\a[24] ), .out0(new_n228));
  tech160nm_fiaoi012aa1n02p5x5 g133(.a(new_n228), .b(new_n227), .c(new_n225), .o1(new_n229));
  nanp03aa1n03x5               g134(.a(new_n227), .b(new_n225), .c(new_n228), .o1(new_n230));
  norb02aa1n02x7               g135(.a(new_n230), .b(new_n229), .out0(\s[24] ));
  norp02aa1n04x5               g136(.a(new_n228), .b(new_n221), .o1(new_n232));
  nano22aa1n02x5               g137(.a(new_n199), .b(new_n215), .c(new_n232), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n202), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n215), .b(new_n234), .c(new_n198), .d(new_n190), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n232), .o1(new_n236));
  oao003aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .c(new_n225), .carry(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n236), .c(new_n235), .d(new_n218), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[24] ), .b(\a[25] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n238), .c(new_n166), .d(new_n233), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n240), .b(new_n238), .c(new_n166), .d(new_n233), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n241), .b(new_n242), .out0(\s[25] ));
  nor042aa1n03x5               g148(.a(\b[24] ), .b(\a[25] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[25] ), .b(\a[26] ), .out0(new_n246));
  tech160nm_fiaoi012aa1n02p5x5 g151(.a(new_n246), .b(new_n241), .c(new_n245), .o1(new_n247));
  nand43aa1n02x5               g152(.a(new_n241), .b(new_n245), .c(new_n246), .o1(new_n248));
  norb02aa1n03x4               g153(.a(new_n248), .b(new_n247), .out0(\s[26] ));
  nor042aa1n04x5               g154(.a(new_n246), .b(new_n239), .o1(new_n250));
  nano32aa1n03x7               g155(.a(new_n199), .b(new_n250), .c(new_n215), .d(new_n232), .out0(new_n251));
  oai012aa1n06x5               g156(.a(new_n251), .b(new_n177), .c(new_n173), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .c(new_n245), .carry(new_n253));
  aobi12aa1n06x5               g158(.a(new_n253), .b(new_n238), .c(new_n250), .out0(new_n254));
  xorc02aa1n12x5               g159(.a(\a[27] ), .b(\b[26] ), .out0(new_n255));
  xnbna2aa1n03x5               g160(.a(new_n255), .b(new_n252), .c(new_n254), .out0(\s[27] ));
  norp02aa1n02x5               g161(.a(\b[26] ), .b(\a[27] ), .o1(new_n257));
  inv040aa1n03x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n04x5               g163(.a(new_n232), .b(new_n219), .c(new_n203), .d(new_n215), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n250), .o1(new_n260));
  aoai13aa1n04x5               g165(.a(new_n253), .b(new_n260), .c(new_n259), .d(new_n237), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n255), .b(new_n261), .c(new_n166), .d(new_n251), .o1(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[27] ), .b(\a[28] ), .out0(new_n263));
  aoi012aa1n03x5               g168(.a(new_n263), .b(new_n262), .c(new_n258), .o1(new_n264));
  aobi12aa1n03x5               g169(.a(new_n255), .b(new_n252), .c(new_n254), .out0(new_n265));
  nano22aa1n03x5               g170(.a(new_n265), .b(new_n258), .c(new_n263), .out0(new_n266));
  nor002aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[28] ));
  norb02aa1n02x5               g172(.a(new_n255), .b(new_n263), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n261), .c(new_n166), .d(new_n251), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[28] ), .b(\b[27] ), .c(new_n258), .carry(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[28] ), .b(\a[29] ), .out0(new_n271));
  aoi012aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n270), .o1(new_n272));
  aobi12aa1n02x7               g177(.a(new_n268), .b(new_n252), .c(new_n254), .out0(new_n273));
  nano22aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n271), .out0(new_n274));
  nor002aa1n02x5               g179(.a(new_n272), .b(new_n274), .o1(\s[29] ));
  xorb03aa1n02x5               g180(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g181(.a(new_n255), .b(new_n271), .c(new_n263), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n261), .c(new_n166), .d(new_n251), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[29] ), .b(\b[28] ), .c(new_n270), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[29] ), .b(\a[30] ), .out0(new_n280));
  aoi012aa1n03x5               g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  aobi12aa1n03x5               g186(.a(new_n277), .b(new_n252), .c(new_n254), .out0(new_n282));
  nano22aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n280), .out0(new_n283));
  nor002aa1n02x5               g188(.a(new_n281), .b(new_n283), .o1(\s[30] ));
  xnrc02aa1n02x5               g189(.a(\b[30] ), .b(\a[31] ), .out0(new_n285));
  norb02aa1n03x4               g190(.a(new_n277), .b(new_n280), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n261), .c(new_n166), .d(new_n251), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[30] ), .b(\b[29] ), .c(new_n279), .carry(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n285), .b(new_n287), .c(new_n288), .o1(new_n289));
  aobi12aa1n03x5               g194(.a(new_n286), .b(new_n252), .c(new_n254), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n290), .b(new_n285), .c(new_n288), .out0(new_n291));
  nor002aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[31] ));
  inv000aa1d42x5               g197(.a(\a[3] ), .o1(new_n293));
  xorb03aa1n02x5               g198(.a(new_n101), .b(\b[2] ), .c(new_n293), .out0(\s[3] ));
  norp02aa1n02x5               g199(.a(new_n97), .b(new_n101), .o1(new_n295));
  xorc02aa1n02x5               g200(.a(\a[4] ), .b(\b[3] ), .out0(new_n296));
  aoib12aa1n02x5               g201(.a(new_n296), .b(new_n293), .c(\b[2] ), .out0(new_n297));
  aboi22aa1n03x5               g202(.a(new_n295), .b(new_n297), .c(new_n103), .d(new_n296), .out0(\s[4] ));
  xnbna2aa1n03x5               g203(.a(new_n111), .b(new_n103), .c(new_n107), .out0(\s[5] ));
  aoi013aa1n02x4               g204(.a(new_n109), .b(new_n103), .c(new_n107), .d(new_n110), .o1(new_n300));
  xnrb03aa1n02x5               g205(.a(new_n300), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g206(.a(\a[6] ), .b(\b[5] ), .c(new_n300), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g208(.a(new_n113), .b(new_n302), .c(new_n114), .o1(new_n304));
  xnrb03aa1n02x5               g209(.a(new_n304), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g210(.a(new_n119), .b(new_n137), .out0(\s[9] ));
endmodule


