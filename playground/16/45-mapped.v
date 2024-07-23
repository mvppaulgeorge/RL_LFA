// Benchmark "adder" written by ABC on Wed Jul 17 20:34:39 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n209, new_n210, new_n211, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n228, new_n229, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n243, new_n244, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n299, new_n300, new_n301, new_n303, new_n305,
    new_n307, new_n309;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n12x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n12x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1n16x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv040aa1d32x5               g006(.a(\a[3] ), .o1(new_n102));
  inv040aa1d28x5               g007(.a(\b[2] ), .o1(new_n103));
  nand02aa1d04x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand22aa1n03x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  nanp02aa1n04x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nor042aa1n04x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nand22aa1n12x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\a[4] ), .o1(new_n111));
  aboi22aa1d24x5               g016(.a(\b[3] ), .b(new_n111), .c(new_n102), .d(new_n103), .out0(new_n112));
  oai012aa1n09x5               g017(.a(new_n112), .b(new_n110), .c(new_n106), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  oai012aa1n02x5               g019(.a(new_n114), .b(\b[7] ), .c(\a[8] ), .o1(new_n115));
  aoi022aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  norp02aa1n04x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nona22aa1n03x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .out0(new_n119));
  tech160nm_finand02aa1n05x5   g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  nor042aa1n06x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nand42aa1n03x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nanb03aa1n03x5               g027(.a(new_n121), .b(new_n122), .c(new_n120), .out0(new_n123));
  nor043aa1n03x5               g028(.a(new_n119), .b(new_n123), .c(new_n115), .o1(new_n124));
  oaih22aa1d12x5               g029(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n125));
  nand22aa1n04x5               g030(.a(\b[7] ), .b(\a[8] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(new_n125), .b(new_n126), .o1(new_n127));
  inv040aa1d32x5               g032(.a(\a[7] ), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\b[6] ), .o1(new_n129));
  oai112aa1n03x5               g034(.a(new_n126), .b(new_n120), .c(new_n129), .d(new_n128), .o1(new_n130));
  nor002aa1n03x5               g035(.a(new_n121), .b(new_n118), .o1(new_n131));
  oai013aa1n03x5               g036(.a(new_n127), .b(new_n130), .c(new_n131), .d(new_n125), .o1(new_n132));
  nand42aa1n03x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  norb02aa1n03x4               g038(.a(new_n133), .b(new_n100), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n124), .d(new_n113), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n99), .b(new_n135), .c(new_n101), .out0(\s[10] ));
  inv000aa1d42x5               g041(.a(new_n99), .o1(new_n137));
  oai012aa1n12x5               g042(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n137), .c(new_n135), .d(new_n101), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n12x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand02aa1n04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n143));
  xnrb03aa1n02x5               g048(.a(new_n143), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n12x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nand02aa1d06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nona23aa1d18x5               g051(.a(new_n146), .b(new_n142), .c(new_n141), .d(new_n145), .out0(new_n147));
  nano22aa1n03x7               g052(.a(new_n147), .b(new_n99), .c(new_n134), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n132), .c(new_n124), .d(new_n113), .o1(new_n149));
  aoi012aa1d18x5               g054(.a(new_n145), .b(new_n141), .c(new_n146), .o1(new_n150));
  oai012aa1d24x5               g055(.a(new_n150), .b(new_n147), .c(new_n138), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n149), .b(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n12x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand02aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nona23aa1n09x5               g065(.a(new_n160), .b(new_n156), .c(new_n155), .d(new_n159), .out0(new_n161));
  oai012aa1n02x7               g066(.a(new_n160), .b(new_n159), .c(new_n155), .o1(new_n162));
  aoai13aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n149), .d(new_n152), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g069(.a(\a[16] ), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xnrc02aa1n12x5               g071(.a(\b[14] ), .b(\a[15] ), .out0(new_n167));
  aoib12aa1n02x5               g072(.a(new_n166), .b(new_n163), .c(new_n167), .out0(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[15] ), .c(new_n165), .out0(\s[16] ));
  aoi012aa1n12x5               g074(.a(new_n132), .b(new_n124), .c(new_n113), .o1(new_n170));
  xnrc02aa1n12x5               g075(.a(\b[15] ), .b(\a[16] ), .out0(new_n171));
  nor043aa1n06x5               g076(.a(new_n161), .b(new_n167), .c(new_n171), .o1(new_n172));
  nand22aa1n03x5               g077(.a(new_n172), .b(new_n148), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n174));
  aoib12aa1n02x5               g079(.a(new_n174), .b(new_n165), .c(\b[15] ), .out0(new_n175));
  oai013aa1n03x5               g080(.a(new_n175), .b(new_n167), .c(new_n171), .d(new_n162), .o1(new_n176));
  aoi012aa1n12x5               g081(.a(new_n176), .b(new_n151), .c(new_n172), .o1(new_n177));
  oai012aa1n12x5               g082(.a(new_n177), .b(new_n170), .c(new_n173), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g084(.a(\b[16] ), .b(\a[17] ), .o1(new_n180));
  nand42aa1n03x5               g085(.a(\b[16] ), .b(\a[17] ), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n180), .b(new_n178), .c(new_n181), .o1(new_n182));
  xnrb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1n12x5               g088(.a(\b[17] ), .b(\a[18] ), .o1(new_n184));
  tech160nm_finand02aa1n05x5   g089(.a(\b[17] ), .b(\a[18] ), .o1(new_n185));
  nano23aa1n06x5               g090(.a(new_n180), .b(new_n184), .c(new_n185), .d(new_n181), .out0(new_n186));
  tech160nm_fioai012aa1n05x5   g091(.a(new_n185), .b(new_n184), .c(new_n180), .o1(new_n187));
  inv020aa1n04x5               g092(.a(new_n187), .o1(new_n188));
  nor002aa1d32x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nand22aa1n09x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n188), .c(new_n178), .d(new_n186), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n178), .d(new_n186), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g100(.a(new_n189), .o1(new_n196));
  norp02aa1n12x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nand22aa1n09x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n192), .c(new_n196), .out0(\s[20] ));
  nona23aa1n03x5               g105(.a(new_n198), .b(new_n190), .c(new_n189), .d(new_n197), .out0(new_n201));
  norb02aa1n06x5               g106(.a(new_n186), .b(new_n201), .out0(new_n202));
  aoi012aa1n02x7               g107(.a(new_n197), .b(new_n189), .c(new_n198), .o1(new_n203));
  oai012aa1n02x5               g108(.a(new_n203), .b(new_n201), .c(new_n187), .o1(new_n204));
  xorc02aa1n02x5               g109(.a(\a[21] ), .b(\b[20] ), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n178), .d(new_n202), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n205), .b(new_n204), .c(new_n178), .d(new_n202), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(\s[21] ));
  nor042aa1d18x5               g113(.a(\b[20] ), .b(\a[21] ), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  xnrc02aa1n12x5               g115(.a(\b[21] ), .b(\a[22] ), .out0(new_n211));
  xobna2aa1n03x5               g116(.a(new_n211), .b(new_n206), .c(new_n210), .out0(\s[22] ));
  nano23aa1n06x5               g117(.a(new_n189), .b(new_n197), .c(new_n198), .d(new_n190), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  nano22aa1n12x5               g119(.a(new_n211), .b(new_n210), .c(new_n214), .out0(new_n215));
  nanp03aa1n02x5               g120(.a(new_n215), .b(new_n186), .c(new_n213), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\a[22] ), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\b[21] ), .o1(new_n218));
  oaoi03aa1n02x5               g123(.a(new_n217), .b(new_n218), .c(new_n209), .o1(new_n219));
  aobi12aa1n02x5               g124(.a(new_n219), .b(new_n204), .c(new_n215), .out0(new_n220));
  oaib12aa1n06x5               g125(.a(new_n220), .b(new_n216), .c(new_n178), .out0(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g127(.a(\b[22] ), .b(\a[23] ), .o1(new_n223));
  xorc02aa1n12x5               g128(.a(\a[23] ), .b(\b[22] ), .out0(new_n224));
  xorc02aa1n12x5               g129(.a(\a[24] ), .b(\b[23] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n02x7               g131(.a(new_n226), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  nand02aa1n03x5               g132(.a(new_n221), .b(new_n224), .o1(new_n228));
  nona22aa1n03x5               g133(.a(new_n228), .b(new_n226), .c(new_n223), .out0(new_n229));
  nanp02aa1n03x5               g134(.a(new_n229), .b(new_n227), .o1(\s[24] ));
  inv020aa1n02x5               g135(.a(new_n203), .o1(new_n231));
  aoai13aa1n06x5               g136(.a(new_n215), .b(new_n231), .c(new_n213), .d(new_n188), .o1(new_n232));
  nand02aa1d04x5               g137(.a(new_n225), .b(new_n224), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\a[24] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\b[23] ), .o1(new_n235));
  oao003aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n223), .carry(new_n236));
  inv000aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n12x5               g142(.a(new_n237), .b(new_n233), .c(new_n232), .d(new_n219), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  nona22aa1n03x5               g144(.a(new_n178), .b(new_n216), .c(new_n233), .out0(new_n240));
  xorc02aa1n12x5               g145(.a(\a[25] ), .b(\b[24] ), .out0(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n240), .c(new_n239), .out0(\s[25] ));
  nand02aa1n04x5               g147(.a(new_n240), .b(new_n239), .o1(new_n243));
  norp02aa1n02x5               g148(.a(\b[24] ), .b(\a[25] ), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[26] ), .b(\b[25] ), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n246), .b(new_n244), .c(new_n243), .d(new_n241), .o1(new_n247));
  nand22aa1n03x5               g152(.a(new_n243), .b(new_n241), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n248), .b(new_n246), .c(new_n244), .out0(new_n249));
  nanp02aa1n02x5               g154(.a(new_n249), .b(new_n247), .o1(\s[26] ));
  inv000aa1d42x5               g155(.a(\a[27] ), .o1(new_n251));
  and002aa1n02x5               g156(.a(new_n245), .b(new_n241), .o(new_n252));
  nano22aa1n03x7               g157(.a(new_n233), .b(new_n241), .c(new_n245), .out0(new_n253));
  nand23aa1n09x5               g158(.a(new_n253), .b(new_n202), .c(new_n215), .o1(new_n254));
  oaoi13aa1n09x5               g159(.a(new_n254), .b(new_n177), .c(new_n170), .d(new_n173), .o1(new_n255));
  inv000aa1d42x5               g160(.a(\a[26] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(\b[25] ), .o1(new_n257));
  oao003aa1n02x5               g162(.a(new_n256), .b(new_n257), .c(new_n244), .carry(new_n258));
  aoi112aa1n06x5               g163(.a(new_n255), .b(new_n258), .c(new_n238), .d(new_n252), .o1(new_n259));
  xorb03aa1n03x5               g164(.a(new_n259), .b(\b[26] ), .c(new_n251), .out0(\s[27] ));
  nanp02aa1n09x5               g165(.a(new_n238), .b(new_n252), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n254), .o1(new_n262));
  nand02aa1d04x5               g167(.a(new_n178), .b(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n258), .o1(new_n264));
  nanp03aa1n06x5               g169(.a(new_n263), .b(new_n261), .c(new_n264), .o1(new_n265));
  norp02aa1n02x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  xorc02aa1n02x5               g171(.a(\a[27] ), .b(\b[26] ), .out0(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[27] ), .b(\a[28] ), .out0(new_n268));
  aoai13aa1n02x7               g173(.a(new_n268), .b(new_n266), .c(new_n265), .d(new_n267), .o1(new_n269));
  nand42aa1n02x5               g174(.a(new_n265), .b(new_n267), .o1(new_n270));
  nona22aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .out0(new_n271));
  nanp02aa1n03x5               g176(.a(new_n271), .b(new_n269), .o1(\s[28] ));
  inv000aa1d42x5               g177(.a(\a[28] ), .o1(new_n273));
  xroi22aa1d04x5               g178(.a(new_n251), .b(\b[26] ), .c(new_n273), .d(\b[27] ), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aoi112aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n276));
  aoib12aa1n02x5               g181(.a(new_n276), .b(new_n273), .c(\b[27] ), .out0(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  oaoi13aa1n03x5               g183(.a(new_n278), .b(new_n277), .c(new_n259), .d(new_n275), .o1(new_n279));
  aoi013aa1n03x5               g184(.a(new_n275), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n280));
  nano22aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n278), .out0(new_n281));
  norp02aa1n03x5               g186(.a(new_n279), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nona22aa1n02x4               g188(.a(new_n267), .b(new_n268), .c(new_n278), .out0(new_n284));
  oao003aa1n02x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  oaoi13aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n259), .d(new_n284), .o1(new_n287));
  aoi013aa1n03x5               g192(.a(new_n284), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[30] ));
  xnrc02aa1n02x5               g195(.a(\b[30] ), .b(\a[31] ), .out0(new_n291));
  nona22aa1n02x4               g196(.a(new_n274), .b(new_n278), .c(new_n286), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .carry(new_n293));
  oaoi13aa1n03x5               g198(.a(new_n291), .b(new_n293), .c(new_n259), .d(new_n292), .o1(new_n294));
  aoi013aa1n03x5               g199(.a(new_n292), .b(new_n263), .c(new_n261), .d(new_n264), .o1(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n293), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[31] ));
  xnbna2aa1n03x5               g202(.a(new_n110), .b(new_n104), .c(new_n105), .out0(\s[3] ));
  nanb02aa1n02x5               g203(.a(\b[3] ), .b(new_n111), .out0(new_n299));
  aob012aa1n02x5               g204(.a(new_n104), .b(new_n299), .c(new_n114), .out0(new_n300));
  oab012aa1n02x4               g205(.a(new_n300), .b(new_n106), .c(new_n110), .out0(new_n301));
  aoi013aa1n02x4               g206(.a(new_n301), .b(new_n113), .c(new_n114), .d(new_n299), .o1(\s[4] ));
  nanb02aa1n02x5               g207(.a(new_n121), .b(new_n122), .out0(new_n303));
  xnbna2aa1n03x5               g208(.a(new_n303), .b(new_n113), .c(new_n114), .out0(\s[5] ));
  aoi013aa1n02x4               g209(.a(new_n121), .b(new_n113), .c(new_n114), .d(new_n122), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g211(.a(\a[6] ), .b(\b[5] ), .c(new_n305), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g213(.a(new_n128), .b(new_n129), .c(new_n307), .o1(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g215(.a(new_n170), .b(new_n133), .c(new_n101), .out0(\s[9] ));
endmodule


