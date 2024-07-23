// Benchmark "adder" written by ABC on Thu Jul 18 02:20:22 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n308, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n317, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n327, new_n330, new_n331, new_n332, new_n333,
    new_n335, new_n337;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n02x5               g001(.a(\b[3] ), .b(\a[4] ), .o1(new_n97));
  nanp02aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nor022aa1n04x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nona23aa1n03x5               g005(.a(new_n100), .b(new_n98), .c(new_n97), .d(new_n99), .out0(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor002aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  tech160nm_fioai012aa1n05x5   g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  aoi012aa1n02x5               g010(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n106));
  oai012aa1n09x5               g011(.a(new_n106), .b(new_n101), .c(new_n105), .o1(new_n107));
  nor022aa1n06x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor022aa1n12x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nona23aa1n03x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  xnrc02aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  nor043aa1n03x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  aoi012aa1n02x5               g023(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n120));
  tech160nm_fioai012aa1n04x5   g025(.a(new_n120), .b(new_n112), .c(new_n119), .o1(new_n121));
  aoi012aa1n12x5               g026(.a(new_n121), .b(new_n107), .c(new_n115), .o1(new_n122));
  oaoi03aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .c(new_n122), .o1(new_n123));
  xorb03aa1n02x5               g028(.a(new_n123), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n04x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand22aa1n04x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nor042aa1n06x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nona23aa1n02x4               g033(.a(new_n128), .b(new_n126), .c(new_n125), .d(new_n127), .out0(new_n129));
  aoi012aa1d24x5               g034(.a(new_n125), .b(new_n127), .c(new_n126), .o1(new_n130));
  tech160nm_fioai012aa1n05x5   g035(.a(new_n130), .b(new_n122), .c(new_n129), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv040aa1n08x5               g038(.a(new_n133), .o1(new_n134));
  nand02aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n06x4               g040(.a(new_n135), .b(new_n133), .out0(new_n136));
  nanp02aa1n02x5               g041(.a(new_n131), .b(new_n136), .o1(new_n137));
  nor022aa1n16x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n12x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n134), .out0(\s[12] ));
  nona23aa1n09x5               g046(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n138), .out0(new_n142));
  oaoi03aa1n12x5               g047(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n143));
  inv040aa1n02x5               g048(.a(new_n143), .o1(new_n144));
  oai012aa1n12x5               g049(.a(new_n144), .b(new_n142), .c(new_n130), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  oai013aa1n03x5               g051(.a(new_n146), .b(new_n122), .c(new_n129), .d(new_n142), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n12x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n04x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n03x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  aoi112aa1n03x5               g057(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n153));
  oai112aa1n06x5               g058(.a(new_n136), .b(new_n140), .c(new_n153), .d(new_n125), .o1(new_n154));
  nor042aa1n06x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand22aa1n12x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nano23aa1n09x5               g061(.a(new_n149), .b(new_n155), .c(new_n156), .d(new_n150), .out0(new_n157));
  inv020aa1n04x5               g062(.a(new_n157), .o1(new_n158));
  aoi012aa1d24x5               g063(.a(new_n155), .b(new_n149), .c(new_n156), .o1(new_n159));
  aoai13aa1n12x5               g064(.a(new_n159), .b(new_n158), .c(new_n154), .d(new_n144), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  inv000aa1n06x5               g066(.a(new_n122), .o1(new_n162));
  nona32aa1n02x4               g067(.a(new_n162), .b(new_n158), .c(new_n142), .d(new_n129), .out0(new_n163));
  nand42aa1n03x5               g068(.a(new_n163), .b(new_n161), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1d28x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n04x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1n16x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(new_n166), .b(new_n170), .c(new_n164), .d(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(\s[16] ));
  norb02aa1n02x5               g078(.a(new_n126), .b(new_n125), .out0(new_n174));
  norb02aa1n03x5               g079(.a(new_n128), .b(new_n127), .out0(new_n175));
  nano23aa1n03x7               g080(.a(new_n133), .b(new_n138), .c(new_n139), .d(new_n135), .out0(new_n176));
  nano23aa1d15x5               g081(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n177));
  nand02aa1d04x5               g082(.a(new_n177), .b(new_n157), .o1(new_n178));
  nano32aa1n03x7               g083(.a(new_n178), .b(new_n176), .c(new_n175), .d(new_n174), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n121), .c(new_n107), .d(new_n115), .o1(new_n180));
  oai012aa1n02x5               g085(.a(new_n169), .b(new_n168), .c(new_n166), .o1(new_n181));
  aobi12aa1n12x5               g086(.a(new_n181), .b(new_n160), .c(new_n177), .out0(new_n182));
  nor042aa1n02x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  nand42aa1n03x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  norb02aa1n03x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n182), .c(new_n180), .out0(\s[17] ));
  tech160nm_fixorc02aa1n03p5x5 g091(.a(\a[18] ), .b(\b[17] ), .out0(new_n187));
  oai112aa1n02x5               g092(.a(new_n182), .b(new_n180), .c(\b[16] ), .d(\a[17] ), .o1(new_n188));
  xobna2aa1n03x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .out0(\s[18] ));
  inv000aa1d42x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  xroi22aa1d04x5               g096(.a(new_n190), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[17] ), .o1(new_n194));
  oaoi03aa1n02x5               g099(.a(new_n191), .b(new_n194), .c(new_n183), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n193), .c(new_n182), .d(new_n180), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n16x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand22aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n09x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  nor042aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand22aa1n04x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n06x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoi112aa1n03x4               g111(.a(new_n199), .b(new_n206), .c(new_n196), .d(new_n202), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n199), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n159), .o1(new_n209));
  aoai13aa1n09x5               g114(.a(new_n177), .b(new_n209), .c(new_n145), .d(new_n157), .o1(new_n210));
  nanp03aa1d12x5               g115(.a(new_n180), .b(new_n210), .c(new_n181), .o1(new_n211));
  oao003aa1n02x5               g116(.a(new_n191), .b(new_n194), .c(new_n183), .carry(new_n212));
  aoai13aa1n03x5               g117(.a(new_n202), .b(new_n212), .c(new_n211), .d(new_n192), .o1(new_n213));
  tech160nm_fiaoi012aa1n05x5   g118(.a(new_n205), .b(new_n213), .c(new_n208), .o1(new_n214));
  norp02aa1n03x5               g119(.a(new_n214), .b(new_n207), .o1(\s[20] ));
  nano23aa1n06x5               g120(.a(new_n199), .b(new_n203), .c(new_n204), .d(new_n200), .out0(new_n216));
  nanp03aa1d12x5               g121(.a(new_n216), .b(new_n185), .c(new_n187), .o1(new_n217));
  oai012aa1n02x5               g122(.a(new_n204), .b(new_n203), .c(new_n199), .o1(new_n218));
  aobi12aa1n06x5               g123(.a(new_n218), .b(new_n216), .c(new_n212), .out0(new_n219));
  aoai13aa1n02x7               g124(.a(new_n219), .b(new_n217), .c(new_n182), .d(new_n180), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n09x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xorc02aa1n12x5               g129(.a(\a[22] ), .b(\b[21] ), .out0(new_n225));
  aoi112aa1n03x4               g130(.a(new_n222), .b(new_n225), .c(new_n220), .d(new_n224), .o1(new_n226));
  inv000aa1n03x5               g131(.a(new_n222), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n217), .o1(new_n228));
  oai013aa1n03x5               g133(.a(new_n218), .b(new_n195), .c(new_n201), .d(new_n205), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n224), .b(new_n229), .c(new_n211), .d(new_n228), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n225), .o1(new_n231));
  tech160nm_fiaoi012aa1n05x5   g136(.a(new_n231), .b(new_n230), .c(new_n227), .o1(new_n232));
  nor002aa1n02x5               g137(.a(new_n232), .b(new_n226), .o1(\s[22] ));
  inv000aa1d42x5               g138(.a(new_n130), .o1(new_n234));
  aoai13aa1n02x5               g139(.a(new_n157), .b(new_n143), .c(new_n176), .d(new_n234), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n177), .o1(new_n236));
  aoai13aa1n02x5               g141(.a(new_n181), .b(new_n236), .c(new_n235), .d(new_n159), .o1(new_n237));
  nanp02aa1n02x5               g142(.a(new_n225), .b(new_n224), .o1(new_n238));
  nano32aa1n02x4               g143(.a(new_n238), .b(new_n216), .c(new_n187), .d(new_n185), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n237), .c(new_n162), .d(new_n179), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n238), .o1(new_n241));
  oaoi03aa1n02x5               g146(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .o1(new_n242));
  aoi012aa1n06x5               g147(.a(new_n242), .b(new_n229), .c(new_n241), .o1(new_n243));
  nor042aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  nand22aa1n04x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  nanb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(new_n246));
  xobna2aa1n03x5               g151(.a(new_n246), .b(new_n240), .c(new_n243), .out0(\s[23] ));
  nor042aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  nand42aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n243), .o1(new_n251));
  aoi112aa1n03x5               g156(.a(new_n244), .b(new_n251), .c(new_n211), .d(new_n239), .o1(new_n252));
  nano22aa1n03x7               g157(.a(new_n252), .b(new_n245), .c(new_n250), .out0(new_n253));
  nona22aa1n09x5               g158(.a(new_n240), .b(new_n251), .c(new_n244), .out0(new_n254));
  tech160nm_fiaoi012aa1n02p5x5 g159(.a(new_n250), .b(new_n254), .c(new_n245), .o1(new_n255));
  norp02aa1n03x5               g160(.a(new_n255), .b(new_n253), .o1(\s[24] ));
  nano23aa1n06x5               g161(.a(new_n244), .b(new_n248), .c(new_n249), .d(new_n245), .out0(new_n257));
  nano32aa1n03x7               g162(.a(new_n217), .b(new_n257), .c(new_n224), .d(new_n225), .out0(new_n258));
  inv000aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  nand23aa1n04x5               g164(.a(new_n257), .b(new_n224), .c(new_n225), .o1(new_n260));
  aoi112aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n261));
  aoi112aa1n06x5               g166(.a(new_n261), .b(new_n248), .c(new_n257), .d(new_n242), .o1(new_n262));
  oai012aa1n18x5               g167(.a(new_n262), .b(new_n219), .c(new_n260), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n259), .c(new_n182), .d(new_n180), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  xorc02aa1n12x5               g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  aoi112aa1n03x4               g174(.a(new_n267), .b(new_n269), .c(new_n265), .d(new_n268), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n267), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n268), .b(new_n263), .c(new_n211), .d(new_n258), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n269), .o1(new_n273));
  tech160nm_fiaoi012aa1n02p5x5 g178(.a(new_n273), .b(new_n272), .c(new_n271), .o1(new_n274));
  nor002aa1n02x5               g179(.a(new_n274), .b(new_n270), .o1(\s[26] ));
  nanp02aa1n02x5               g180(.a(new_n269), .b(new_n268), .o1(new_n276));
  nano23aa1d12x5               g181(.a(new_n217), .b(new_n276), .c(new_n241), .d(new_n257), .out0(new_n277));
  inv020aa1n02x5               g182(.a(new_n277), .o1(new_n278));
  and002aa1n02x5               g183(.a(new_n269), .b(new_n268), .o(new_n279));
  oaoi03aa1n12x5               g184(.a(\a[26] ), .b(\b[25] ), .c(new_n271), .o1(new_n280));
  aoi012aa1n06x5               g185(.a(new_n280), .b(new_n263), .c(new_n279), .o1(new_n281));
  aoai13aa1n04x5               g186(.a(new_n281), .b(new_n278), .c(new_n182), .d(new_n180), .o1(new_n282));
  xorb03aa1n03x5               g187(.a(new_n282), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nanp02aa1n02x5               g188(.a(\b[26] ), .b(\a[27] ), .o1(new_n284));
  xorc02aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n280), .o1(new_n286));
  aob012aa1d18x5               g191(.a(new_n286), .b(new_n263), .c(new_n279), .out0(new_n287));
  nor002aa1d32x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  aoi112aa1n03x5               g193(.a(new_n287), .b(new_n288), .c(new_n211), .d(new_n277), .o1(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n284), .c(new_n285), .out0(new_n290));
  aoai13aa1n02x5               g195(.a(new_n277), .b(new_n237), .c(new_n162), .d(new_n179), .o1(new_n291));
  oaoi13aa1n02x5               g196(.a(new_n276), .b(new_n262), .c(new_n219), .d(new_n260), .o1(new_n292));
  nona32aa1n02x4               g197(.a(new_n291), .b(new_n288), .c(new_n280), .d(new_n292), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n285), .b(new_n293), .c(new_n284), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n294), .b(new_n290), .o1(\s[28] ));
  nano22aa1n02x4               g200(.a(new_n288), .b(new_n285), .c(new_n284), .out0(new_n296));
  aoai13aa1n04x5               g201(.a(new_n296), .b(new_n287), .c(new_n211), .d(new_n277), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\a[28] ), .o1(new_n298));
  inv000aa1d42x5               g203(.a(\b[27] ), .o1(new_n299));
  oaoi03aa1n12x5               g204(.a(new_n298), .b(new_n299), .c(new_n288), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[29] ), .b(\b[28] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  tech160nm_fiaoi012aa1n03p5x5 g207(.a(new_n302), .b(new_n297), .c(new_n300), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n300), .o1(new_n304));
  aoi112aa1n03x4               g209(.a(new_n301), .b(new_n304), .c(new_n282), .d(new_n296), .o1(new_n305));
  nor002aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g211(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g212(.a(new_n302), .b(new_n288), .c(new_n285), .d(new_n284), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n287), .c(new_n211), .d(new_n277), .o1(new_n309));
  oaoi03aa1n09x5               g214(.a(\a[29] ), .b(\b[28] ), .c(new_n300), .o1(new_n310));
  inv000aa1n02x5               g215(.a(new_n310), .o1(new_n311));
  tech160nm_fixorc02aa1n03p5x5 g216(.a(\a[30] ), .b(\b[29] ), .out0(new_n312));
  inv000aa1d42x5               g217(.a(new_n312), .o1(new_n313));
  tech160nm_fiaoi012aa1n05x5   g218(.a(new_n313), .b(new_n309), .c(new_n311), .o1(new_n314));
  aoi112aa1n03x4               g219(.a(new_n312), .b(new_n310), .c(new_n282), .d(new_n308), .o1(new_n315));
  norp02aa1n03x5               g220(.a(new_n314), .b(new_n315), .o1(\s[30] ));
  and003aa1n02x5               g221(.a(new_n296), .b(new_n312), .c(new_n301), .o(new_n317));
  oaoi03aa1n02x5               g222(.a(\a[30] ), .b(\b[29] ), .c(new_n311), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[31] ), .b(\b[30] ), .out0(new_n319));
  aoi112aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n282), .d(new_n317), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n317), .b(new_n287), .c(new_n211), .d(new_n277), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n318), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n319), .o1(new_n323));
  aoi012aa1n06x5               g228(.a(new_n323), .b(new_n321), .c(new_n322), .o1(new_n324));
  nor042aa1n03x5               g229(.a(new_n324), .b(new_n320), .o1(\s[31] ));
  xnrb03aa1n02x5               g230(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g231(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g233(.a(new_n107), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  and002aa1n02x5               g234(.a(\b[4] ), .b(\a[5] ), .o(new_n330));
  norp02aa1n02x5               g235(.a(new_n107), .b(new_n118), .o1(new_n331));
  norp03aa1n02x5               g236(.a(new_n331), .b(new_n330), .c(new_n113), .o1(new_n332));
  oai012aa1n02x5               g237(.a(new_n113), .b(new_n331), .c(new_n330), .o1(new_n333));
  norb02aa1n02x5               g238(.a(new_n333), .b(new_n332), .out0(\s[6] ));
  norp02aa1n02x5               g239(.a(new_n332), .b(new_n116), .o1(new_n335));
  xnrb03aa1n02x5               g240(.a(new_n335), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi13aa1n02x5               g241(.a(new_n110), .b(new_n111), .c(new_n332), .d(new_n116), .o1(new_n337));
  xnrb03aa1n02x5               g242(.a(new_n337), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g243(.a(new_n122), .b(new_n175), .out0(\s[9] ));
endmodule


